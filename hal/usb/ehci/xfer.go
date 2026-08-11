// EHCI control-transfer layer: async schedule + queue head / qTD mechanics for
// USB device enumeration over the root port.
//
// This drives a single control endpoint (EP0) through the EHCI asynchronous
// schedule: it builds a queue head (QH) and a SETUP/DATA/STATUS chain of queue
// element transfer descriptors (qTDs) in caller-supplied DMA memory, points the
// controller's ASYNCLISTADDR at the QH, enables the async schedule and polls
// the qTD status to completion.
//
// The AST2700 places DRAM at 0x4_0000_0000 (above 4 GB) and the USB DMA master
// addresses it identity-mapped, so the controller is driven in 64-bit
// addressing mode: CTRLDSSEGMENT carries the common high 32 bits for the
// schedule pointers and each qTD's extended buffer pointers carry the high 32
// bits for data buffers. Cache maintenance is explicit (the USB path is not
// cache-coherent) via the injected DMA backend.
package ehci

import (
	"encoding/binary"
	"errors"
	"time"

	"src.kyanite.computer/aspeed-go/reg"
)

// DMA supplies physically-addressable memory and cache maintenance to the
// transfer layer. aspeed-go stays free of any runtime/OS dependency; the caller
// (the bare-metal payload) implements this with its DMA allocator and cache
// primitives. Buffers must be identity-mapped (physical address == the address
// the CPU uses) and remain pinned until Free.
type DMA interface {
	// Alloc returns a zeroed buffer of size bytes aligned to align, plus its
	// physical address (what the controller DMAs to).
	Alloc(size, align int) (buf []byte, phys uint64)
	// Free releases a buffer previously returned by Alloc.
	Free(phys uint64)
	// Clean writes CPU cache lines back to memory; call after the CPU writes a
	// structure/buffer the controller will read.
	Clean(phys uint64, size int)
	// Invalidate refreshes CPU cache lines from memory; call before the CPU
	// reads a structure/buffer the controller wrote.
	Invalidate(phys uint64, size int)
}

// Operational registers used by the async schedule (relative to op base).
const (
	opCTRLDSSEG = 0x10 // CTRLDSSEGMENT: high 32 bits for 64-bit addressing.
	opASYNCADDR = 0x18 // ASYNCLISTADDR: current async-list QH pointer.
)

// USBCMD/USBSTS async-schedule bits (in addition to those in ehci.go).
const (
	cmdIntAsyncAdv = 1 << 6  // interrupt on async advance doorbell.
	stsAsyncStatus = 1 << 15 // USBSTS: async schedule is running.
)

// hccp64bit is HCCPARAMS bit0: 64-bit addressing capability (CTRLDSSEGMENT +
// extended qTD/QH pointers are honoured only when set).
const hccp64bit = 1 << 0

// XferDebug captures controller and descriptor state around a control transfer
// for diagnostics when it fails.
type XferDebug struct {
	HCCParams uint32
	USBCmd    uint32
	USBSts    uint32
	CtrlDSSeg uint32
	AsyncAddr    uint32
	AsyncAddrImm uint32 // ASYNCLISTADDR read back immediately after the write.
	PortSC    uint32
	QHPhys    uint64
	QHLinkMem uint32 // QH horizontal link as it sits in DRAM (post-invalidate).
	QHCharMem uint32 // QH endpoint-characteristics in DRAM.
	QHToken   uint32 // QH overlay token.
	QHCurQTD  uint32 // QH current qTD pointer.
	SetupTok  uint32
	DataTok   uint32
	StatusTok uint32
}

// Addr64 reports whether the controller advertises 64-bit addressing.
func (d XferDebug) Addr64() bool { return d.HCCParams&hccp64bit != 0 }

// AsyncRunning reports whether the async schedule was running.
func (d XferDebug) AsyncRunning() bool { return d.USBSts&stsAsyncStatus != 0 }

// qTD token field bits/shifts.
const (
	qtdStatusActive = 1 << 7
	qtdStatusHalted = 1 << 6
	qtdStatusBufErr = 1 << 5
	qtdStatusBabble = 1 << 4
	qtdStatusXactErr = 1 << 3
	qtdStatusMissed = 1 << 2

	qtdPIDOut   = 0 << 8
	qtdPIDIn    = 1 << 8
	qtdPIDSetup = 2 << 8

	qtdCERR3    = 3 << 10 // error-retry counter = 3.
	qtdIOC      = 1 << 15
	qtdBytesShift = 16      // total bytes to transfer [30:16].
	qtdToggle   = 1 << 31

	qtdErrMask = qtdStatusHalted | qtdStatusBufErr | qtdStatusBabble | qtdStatusXactErr
)

// QH endpoint-characteristics (DW1) / capabilities (DW2) bits.
const (
	qhEPSFull = 0 << 12
	qhEPSLow  = 1 << 12
	qhEPSHigh = 2 << 12
	qhDTC     = 1 << 14 // data-toggle from qTD.
	qhHead    = 1 << 15 // head of reclamation list.
	qhMPSShift = 16     // max packet length [26:16].
	qhControlEP = 1 << 27 // non-HS control endpoint flag.

	qhMult1 = 1 << 30 // DW2 [31:30] = 1 transaction per microframe.

	qhTypQH = 1 << 1 // horizontal-link Typ = QH.
	linkTerminate = 1 << 0
)

// Slot sizes (generous, 32-byte aligned) for the 64-bit descriptor variants.
const (
	qhBytes  = 128
	qtdBytes = 64
	// qTD dword offsets.
	qtdNext     = 0
	qtdAltNext  = 4
	qtdToken    = 8
	qtdBuf0     = 12 // buffer pointers [0..4] at +12,+16,+20,+24,+28.
	qtdExtBuf0  = 32 // ext (high-32) buffer pointers [0..4] at +32..+48.
	// QH dword offsets.
	qhLink      = 0
	qhEndpChar  = 4
	qhEndpCap   = 8
	qhCurQTD    = 12
	qhOverlay   = 16 // transfer overlay: a qTD image (next at +16, token at +24...).
)

// Errors returned by control transfers.
var (
	ErrTimeout = errors.New("ehci: transfer timed out")
	ErrHalted  = errors.New("ehci: qTD halted (transaction error)")
	ErrNoDMA   = errors.New("ehci: no DMA backend configured")
)

// low32 and high32 split a physical address for the EHCI 64-bit pointers.
func low32(p uint64) uint32  { return uint32(p) }
func high32(p uint64) uint32 { return uint32(p >> 32) }

// buildQTD writes a qTD into slot at (buf,phys) with the given PID, toggle,
// data buffer and length, linked to nextPhys (0 => terminate). It returns
// nothing; the caller cleans the cache.
func buildQTD(buf []byte, pid, toggle uint32, dataPhys uint64, length int, nextPhys uint64, ioc bool) {
	for i := range buf[:qtdBytes] {
		buf[i] = 0
	}
	if nextPhys != 0 {
		binary.LittleEndian.PutUint32(buf[qtdNext:], low32(nextPhys)&^0x1f)
	} else {
		binary.LittleEndian.PutUint32(buf[qtdNext:], linkTerminate)
	}
	binary.LittleEndian.PutUint32(buf[qtdAltNext:], linkTerminate)

	token := qtdStatusActive | pid | qtdCERR3 | (uint32(length) << qtdBytesShift)
	if toggle != 0 {
		token |= qtdToggle
	}
	if ioc {
		token |= qtdIOC
	}
	binary.LittleEndian.PutUint32(buf[qtdToken:], token)

	if length > 0 {
		// Buffer pointer 0 (page base + offset) and, in case the buffer crosses
		// a 4 KiB page, pointer 1 (next page). Buffers are page-aligned by the
		// caller so at most two pages are needed for the small descriptors here.
		binary.LittleEndian.PutUint32(buf[qtdBuf0:], low32(dataPhys))
		binary.LittleEndian.PutUint32(buf[qtdExtBuf0:], high32(dataPhys))
		next := (dataPhys &^ 0xfff) + 0x1000
		binary.LittleEndian.PutUint32(buf[qtdBuf0+4:], low32(next))
		binary.LittleEndian.PutUint32(buf[qtdExtBuf0+4:], high32(next))
	}
}

// buildQH writes the control-endpoint queue head into slot (buf,phys), linked
// horizontally to selfPhys (a single-QH async ring points to itself), with the
// overlay pointing at the first qTD.
func buildQH(buf []byte, selfPhys uint64, addr uint8, mps uint16, hs bool, firstQTD uint64) {
	for i := range buf[:qhBytes] {
		buf[i] = 0
	}
	// Horizontal link -> self, Typ=QH.
	binary.LittleEndian.PutUint32(buf[qhLink:], (low32(selfPhys)&^0x1f)|qhTypQH)

	eps := uint32(qhEPSHigh)
	if !hs {
		eps = qhEPSFull
	}
	epchar := uint32(addr) | (0 << 8) /*EP0*/ | eps | qhDTC | qhHead | (uint32(mps) << qhMPSShift)
	if !hs {
		epchar |= qhControlEP // C: non-high-speed control endpoint.
	}
	binary.LittleEndian.PutUint32(buf[qhEndpChar:], epchar)
	binary.LittleEndian.PutUint32(buf[qhEndpCap:], qhMult1)

	// Current qTD = 0; overlay.next -> first qTD, overlay token inactive so the
	// controller fetches the first qTD.
	binary.LittleEndian.PutUint32(buf[qhCurQTD:], 0)
	binary.LittleEndian.PutUint32(buf[qhOverlay+qtdNext:], low32(firstQTD)&^0x1f)
	binary.LittleEndian.PutUint32(buf[qhOverlay+qtdAltNext:], linkTerminate)
	binary.LittleEndian.PutUint32(buf[qhOverlay+qtdToken:], 0)
}

// ControlIn performs a control IN transfer on EP0 of the device at the given
// address: an 8-byte SETUP, an IN data stage into a buffer of len(data), and a
// zero-length OUT status stage. It returns the number of bytes received. The
// device must already be reset/enabled (see ResetPort) and the caller must have
// provided a DMA backend via SetDMA.
func (c *Controller) ControlIn(addr uint8, mps uint16, setup [8]byte, data []byte) (int, error) {
	if c.dma == nil {
		return 0, ErrNoDMA
	}
	d := c.dma

	// Allocate the schedule structures and buffers (page-aligned buffers so a
	// small descriptor never straddles a page in a way needing >2 pointers).
	qhBuf, qhPhys := d.Alloc(qhBytes, 32)
	tdBuf, tdPhys := d.Alloc(qtdBytes*3, 32)
	suBuf, suPhys := d.Alloc(4096, 4096)
	dnBuf, dnPhys := d.Alloc(4096, 4096)
	defer d.Free(qhPhys)
	defer d.Free(tdPhys)
	defer d.Free(suPhys)
	defer d.Free(dnPhys)

	setupPhysQTD := tdPhys
	dataPhysQTD := tdPhys + qtdBytes
	statusPhysQTD := tdPhys + 2*qtdBytes

	// SETUP data.
	copy(suBuf, setup[:])
	d.Clean(suPhys, 8)

	// SETUP -> DATA -> STATUS qTD chain.
	buildQTD(tdBuf[0:qtdBytes], qtdPIDSetup, 0, suPhys, 8, dataPhysQTD, false)
	buildQTD(tdBuf[qtdBytes:2*qtdBytes], qtdPIDIn, qtdToggle, dnPhys, len(data), statusPhysQTD, false)
	buildQTD(tdBuf[2*qtdBytes:3*qtdBytes], qtdPIDOut, qtdToggle, 0, 0, 0, true)
	d.Clean(tdPhys, qtdBytes*3)

	buildQH(qhBuf, qhPhys, addr, mps, true, setupPhysQTD)
	d.Clean(qhPhys, qhBytes)

	// Program the async schedule and run it.
	c.setReg(opCTRLDSSEG, high32(qhPhys))
	c.setReg(opASYNCADDR, low32(qhPhys)&^0x1f)
	asyncImm := c.op(opASYNCADDR)
	c.opw(opUSBCMD, c.op(opUSBCMD)|cmdRunStop|cmdAsyncEn)

	// Poll the STATUS qTD to completion.
	err := c.waitQTD(statusPhysQTD, tdBuf[2*qtdBytes:3*qtdBytes], 1*time.Second)

	// Capture diagnostic state before the schedule is torn down and the buffers
	// are freed, so a failure can be root-caused.
	d.Invalidate(qhPhys, qhBytes)
	d.Invalidate(tdPhys, qtdBytes*3)
	c.dbg = XferDebug{
		HCCParams: reg.Read(c.port.Base + capHCCPARAMS),
		USBCmd:    c.op(opUSBCMD),
		USBSts:    c.op(opUSBSTS),
		CtrlDSSeg: c.op(opCTRLDSSEG),
		AsyncAddr:    c.op(opASYNCADDR),
		AsyncAddrImm: asyncImm,
		PortSC:    c.op(opPORTSC0),
		QHPhys:    qhPhys,
		QHLinkMem: binary.LittleEndian.Uint32(qhBuf[qhLink : qhLink+4]),
		QHCharMem: binary.LittleEndian.Uint32(qhBuf[qhEndpChar : qhEndpChar+4]),
		QHToken:   binary.LittleEndian.Uint32(qhBuf[qhOverlay+qtdToken : qhOverlay+qtdToken+4]),
		QHCurQTD:  binary.LittleEndian.Uint32(qhBuf[qhCurQTD : qhCurQTD+4]),
		SetupTok:  binary.LittleEndian.Uint32(tdBuf[qtdToken : qtdToken+4]),
		DataTok:   binary.LittleEndian.Uint32(tdBuf[qtdBytes+qtdToken : qtdBytes+qtdToken+4]),
		StatusTok: binary.LittleEndian.Uint32(tdBuf[2*qtdBytes+qtdToken : 2*qtdBytes+qtdToken+4]),
	}

	// Stop the async schedule.
	c.opw(opUSBCMD, c.op(opUSBCMD)&^cmdAsyncEn)

	if err != nil {
		return 0, err
	}

	// Bytes received = requested - residual (from the DATA qTD token).
	d.Invalidate(dataPhysQTD, qtdBytes)
	dtoken := binary.LittleEndian.Uint32(tdBuf[qtdBytes+qtdToken : qtdBytes+qtdToken+4])
	residual := int((dtoken >> qtdBytesShift) & 0x7fff)
	got := len(data) - residual
	if got < 0 {
		got = 0
	}
	if got > 0 {
		d.Invalidate(dnPhys, got)
		copy(data, dnBuf[:got])
	}
	return got, nil
}

// waitQTD polls a qTD until its Active bit clears, returning an error if it
// halts or the timeout elapses. tdBuf is the CPU view of the qTD at tdPhys.
func (c *Controller) waitQTD(tdPhys uint64, tdBuf []byte, timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for {
		c.dma.Invalidate(tdPhys, qtdBytes)
		token := binary.LittleEndian.Uint32(tdBuf[qtdToken : qtdToken+4])
		if token&qtdStatusActive == 0 {
			if token&qtdErrMask != 0 {
				return ErrHalted
			}
			return nil
		}
		if !time.Now().Before(deadline) {
			return ErrTimeout
		}
		time.Sleep(time.Millisecond)
	}
}

// setReg / SetDMA helpers.
func (c *Controller) setReg(off, v uint32) { c.opw(off, v) }

// SetDMA installs the DMA backend used by the transfer layer.
func (c *Controller) SetDMA(d DMA) { c.dma = d }

// Debug returns the diagnostic state captured during the last control transfer.
func (c *Controller) Debug() XferDebug { return c.dbg }

// USB standard request/descriptor constants.
const (
	reqDirIn        = 0x80 // device-to-host.
	reqGetDescriptor = 0x06
	reqSetAddress    = 0x05

	descDevice = 1
	descConfig = 2
	descString = 3
)

// SetupPacket builds an 8-byte USB SETUP packet (little-endian on the wire).
func SetupPacket(bmRequestType, bRequest uint8, wValue, wIndex, wLength uint16) [8]byte {
	var s [8]byte
	s[0] = bmRequestType
	s[1] = bRequest
	binary.LittleEndian.PutUint16(s[2:], wValue)
	binary.LittleEndian.PutUint16(s[4:], wIndex)
	binary.LittleEndian.PutUint16(s[6:], wLength)
	return s
}

// GetDeviceDescriptor fetches up to n bytes of the DEVICE descriptor from the
// device at addr using EP0 max-packet-size mps. Reading the first 8 bytes with
// mps=8 is the standard first step of enumeration (it reveals bMaxPacketSize0).
func (c *Controller) GetDeviceDescriptor(addr uint8, mps uint16, n int) ([]byte, error) {
	setup := SetupPacket(reqDirIn, reqGetDescriptor, descDevice<<8, 0, uint16(n))
	buf := make([]byte, n)
	got, err := c.ControlIn(addr, mps, setup, buf)
	if err != nil {
		return nil, err
	}
	return buf[:got], nil
}
