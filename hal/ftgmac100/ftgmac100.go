// Package ftgmac100 provides a poll-mode Faraday FTGMAC100 Ethernet driver.
package ftgmac100

import (
	"encoding/binary"
	"errors"
	"fmt"
	"unsafe"

	"src.kyanite.computer/aspeed-go/reg"
)

const (
	AST2700MAC0 = 0x14050000
	AST2700MAC1 = 0x14060000
	AST2700MAC2 = 0x14070000

	offISR       = 0x000
	offIER       = 0x004
	offMACMADR   = 0x008
	offMACLADR   = 0x00c
	offNPTXPD    = 0x018
	offNPTXRBadR = 0x020
	offRXRBadR   = 0x024
	offAPTC      = 0x034
	offDBLAC     = 0x038
	offRBSR      = 0x04c
	offMACCR     = 0x050
	offTXRBadRHi = 0x17c
	offRXRBadRHi = 0x18c

	intAll = 0x7ff

	maccrTXDMA       = 1 << 0
	maccrRXDMA       = 1 << 1
	maccrTXMAC       = 1 << 2
	maccrRXMAC       = 1 << 3
	maccrRMVLAN      = 1 << 4
	maccrFullDuplex  = 1 << 8
	maccrGigabit     = 1 << 9
	maccrCRCAppend   = 1 << 10
	maccrRXRunt      = 1 << 12
	maccrRXBroadcast = 1 << 17
	maccrFast        = 1 << 19
	maccrRMII        = 1 << 20
	maccrSWRst       = 1 << 31

	descStride = 64
	bufSize    = 1600
	minFrame   = 60
	maxFrame   = 1514
	fcsLen     = 4

	txCount = 4
	rxCount = 16

	txDescEDOTR = 1 << 30
	txDescFTS   = 1 << 29
	txDescLTS   = 1 << 28
	txDescOwn   = 1 << 31

	rxDescEDORR = 1 << 30
	rxDescReady = 1 << 31
	rxDescErr   = 1<<18 | 1<<19 | 1<<20 | 1<<21 | 1<<22

	desc2AddrHighShift = 16

	// DMABytes is the minimum DMA byte size required by [Device.Init].
	DMABytes = txCount*descStride + rxCount*descStride + txCount*bufSize + rxCount*bufSize
)

var (
	ErrDMATooSmall = errors.New("ftgmac100: DMA buffer too small")
	ErrBadFrame    = errors.New("ftgmac100: bad ethernet frame size")
	ErrTXBusy      = errors.New("ftgmac100: transmit descriptor busy")
	ErrReset       = errors.New("ftgmac100: reset timeout")
	ErrBadLink     = errors.New("ftgmac100: unsupported link mode")
)

// Link describes the Ethernet MAC link mode.
type Link struct {
	SpeedMbps  int
	FullDuplex bool
}

// DMAOps provides optional cache maintenance for DMA buffers.
// On hardware with CPU caches, callers must set these to ensure
// DMA coherency. On QEMU or uncached memory they may be left nil.
type DMAOps struct {
	// Clean writes dirty cache lines back to memory (DC CVAC).
	// Called before the MAC reads a TX descriptor or buffer.
	Clean func(start, size uintptr)
	// Invalidate cleans and invalidates cache lines (DC CIVAC).
	// Called before the CPU reads an RX descriptor or buffer
	// written by the MAC's DMA engine.
	Invalidate func(start, size uintptr)
}

// Device is an FTGMAC100 Ethernet MAC using caller-provided DMA memory.
type Device struct {
	Base uint32
	MAC  [6]byte
	Link Link
	DMA  DMAOps

	dmaAddr uint64
	dma     []byte
	txIdx   int
	rxIdx   int
}

// DMASize returns the minimum DMA byte size required by [Device.Init].
func DMASize() int {
	return DMABytes
}

// Init initializes the MAC and descriptor rings.
func (d *Device) Init(dmaAddr uint64, dma []byte) error {
	if len(dma) < DMASize() {
		return ErrDMATooSmall
	}
	d.dmaAddr = dmaAddr
	d.dma = dma[:DMASize()]
	clear(d.dma)

	if err := d.reset(); err != nil {
		return err
	}
	d.writeMAC()

	reg.Write(d.Base+offIER, 0)
	reg.Write(d.Base+offISR, intAll)
	// NOTE: register 0x58 (TM, "Control Mode Register") is left at its reset
	// value to match the vendor u-boot/Linux ftgmac100 driver, which never
	// writes it (a running vendor system reads it back as 0). The datasheet
	// marks these bits as internal-test-only; do not write them without a
	// vendor-confirmed reason.

	for i := 0; i < txCount; i++ {
		if i == txCount-1 {
			d.putDesc0(d.txDesc(i), txDescEDOTR)
		}
	}
	for i := 0; i < rxCount; i++ {
		addr := d.rxBufAddr(i)
		d.putDesc2(d.rxDesc(i), uint32((addr>>32)&0x7)<<desc2AddrHighShift)
		d.putDesc3(d.rxDesc(i), uint32(addr))
		if i == rxCount-1 {
			d.putDesc0(d.rxDesc(i), rxDescEDORR)
		}
	}

	reg.Write(d.Base+offNPTXRBadR, uint32(d.txDescAddr(0)))
	reg.Write(d.Base+offTXRBadRHi, uint32(d.txDescAddr(0)>>32))
	reg.Write(d.Base+offRXRBadR, uint32(d.rxDescAddr(0)))
	reg.Write(d.Base+offRXRBadRHi, uint32(d.rxDescAddr(0)>>32))

	// Match the vendor u-boot ftgmac100 driver: program only the TX/RX
	// descriptor-size fields [19:12] and leave the DMA burst-size fields at
	// their reset value.
	descUnits := uint32(descStride / 8)
	dblac := reg.Read(d.Base+offDBLAC) &^ (uint32(0xff) << 12)
	dblac |= descUnits << 12 // RXDES_SIZE
	dblac |= descUnits << 16 // TXDES_SIZE
	reg.Write(d.Base+offDBLAC, dblac)
	reg.Write(d.Base+offAPTC, 1)
	reg.Write(d.Base+offRBSR, bufSize)

	d.dmaClean(0, len(d.dma))

	maccr, err := d.maccr()
	if err != nil {
		return err
	}
	reg.Write(d.Base+offMACCR, maccr)
	return nil
}

// SetLink updates the MAC speed and duplex mode.
func (d *Device) SetLink(link Link) error {
	d.Link = link
	maccr, err := d.maccr()
	if err != nil {
		return err
	}
	reg.Write(d.Base+offMACCR, maccr)
	return nil
}

// HardwareAddr6 returns the configured MAC address.
func (d *Device) HardwareAddr6() ([6]byte, error) {
	return d.MAC, nil
}

// SendOffsetEthFrame transmits a complete Ethernet frame.
func (d *Device) SendOffsetEthFrame(frame []byte) error {
	if len(frame) > maxFrame || len(frame) == 0 {
		return ErrBadFrame
	}
	d.dmaInvalidate(d.txDescOff(d.txIdx), descStride)
	desc := d.txDesc(d.txIdx)
	if d.desc0(desc)&txDescOwn != 0 {
		return ErrTXBusy
	}
	n := len(frame)
	if n < minFrame {
		n = minFrame
	}
	txbuf := d.txBuf(d.txIdx)
	clear(txbuf[:n])
	copy(txbuf, frame)

	addr := d.txBufAddr(d.txIdx)
	edotr := d.desc0(desc) & txDescEDOTR
	d.putDesc2(desc, uint32((addr>>32)&0x7)<<desc2AddrHighShift)
	d.putDesc3(desc, uint32(addr))
	d.putDesc0(desc, edotr|txDescFTS|txDescLTS|txDescOwn|uint32(n))
	d.dmaClean(d.txDescOff(d.txIdx), descStride)
	d.dmaClean(d.txBufOff(d.txIdx), bufSize)
	reg.Write(d.Base+offNPTXPD, 1)
	d.txIdx = (d.txIdx + 1) % txCount
	return nil
}

// SetEthRecvHandler is unsupported; use [Device.EthPoll].
func (d *Device) SetEthRecvHandler(handler func([]byte)) {}

// EthPoll receives one Ethernet frame into buf if available.
func (d *Device) EthPoll(buf []byte) (ethFrameOff, ethernetBytes int, err error) {
	d.dmaInvalidate(d.rxDescOff(d.rxIdx), descStride)
	desc := d.rxDesc(d.rxIdx)
	d0 := d.desc0(desc)
	if d0&rxDescReady == 0 {
		return 0, 0, nil
	}
	defer func() {
		d.putDesc0(desc, d0&rxDescEDORR)
		d.dmaClean(d.rxDescOff(d.rxIdx), descStride)
		d.rxIdx = (d.rxIdx + 1) % rxCount
	}()
	if d0&rxDescErr != 0 {
		return 0, 0, nil
	}
	n := int(d0 & 0x3fff)
	if n > bufSize {
		return 0, 0, fmt.Errorf("ftgmac100: RX frame too large: %d", n)
	}
	if n >= fcsLen {
		n -= fcsLen
	}
	d.dmaInvalidate(d.rxBufOff(d.rxIdx), bufSize)
	if n > len(buf) {
		n = len(buf)
	}
	copy(buf, d.rxBuf(d.rxIdx)[:n])
	return 0, n, nil
}

// MaxFrameSizeAndOffset returns Ethernet frame capacity and TX frame offset.
func (d *Device) MaxFrameSizeAndOffset() (maxFrameSize int, frameOff int) {
	return maxFrame, 0
}

func (d *Device) reset() error {
	reg.SetBits32(uintptr(d.Base+offMACCR), maccrSWRst)
	for i := 0; i < 1000000; i++ {
		if reg.Read(d.Base+offMACCR)&maccrSWRst == 0 {
			return nil
		}
	}
	return ErrReset
}

func (d *Device) writeMAC() {
	maddr := uint32(d.MAC[0])<<8 | uint32(d.MAC[1])
	laddr := uint32(d.MAC[2])<<24 | uint32(d.MAC[3])<<16 | uint32(d.MAC[4])<<8 | uint32(d.MAC[5])
	reg.Write(d.Base+offMACMADR, maddr)
	reg.Write(d.Base+offMACLADR, laddr)
}

func (d *Device) maccr() (uint32, error) {
	link := d.Link
	if link.SpeedMbps == 0 {
		link = Link{SpeedMbps: 1000, FullDuplex: true}
	}
	maccr := uint32(maccrTXMAC | maccrRXMAC | maccrTXDMA | maccrRXDMA | maccrCRCAppend | maccrRMVLAN | maccrRXRunt | maccrRXBroadcast)
	if link.FullDuplex {
		maccr |= maccrFullDuplex
	}
	switch link.SpeedMbps {
	case 1000:
		maccr |= maccrGigabit
	case 100:
		maccr |= maccrFast
	case 10:
	default:
		return 0, ErrBadLink
	}
	return maccr, nil
}

func (d *Device) txDesc(i int) []byte { return d.dma[i*descStride:][:descStride] }
func (d *Device) rxDesc(i int) []byte { return d.dma[txCount*descStride+i*descStride:][:descStride] }

func (d *Device) txBuf(i int) []byte {
	off := txCount*descStride + rxCount*descStride + i*bufSize
	return d.dma[off:][:bufSize]
}

func (d *Device) rxBuf(i int) []byte {
	off := txCount*descStride + rxCount*descStride + txCount*bufSize + i*bufSize
	return d.dma[off:][:bufSize]
}

func (d *Device) txDescAddr(i int) uint64 { return d.dmaAddr + uint64(i*descStride) }
func (d *Device) rxDescAddr(i int) uint64 { return d.dmaAddr + uint64(txCount*descStride+i*descStride) }

func (d *Device) txBufAddr(i int) uint64 {
	return d.dmaAddr + uint64(txCount*descStride+rxCount*descStride+i*bufSize)
}

func (d *Device) rxBufAddr(i int) uint64 {
	return d.dmaAddr + uint64(txCount*descStride+rxCount*descStride+txCount*bufSize+i*bufSize)
}

func (d *Device) desc0(desc []byte) uint32       { return binary.LittleEndian.Uint32(desc[0:4]) }
func (d *Device) putDesc0(desc []byte, v uint32) { binary.LittleEndian.PutUint32(desc[0:4], v) }
func (d *Device) putDesc2(desc []byte, v uint32) { binary.LittleEndian.PutUint32(desc[8:12], v) }
func (d *Device) putDesc3(desc []byte, v uint32) { binary.LittleEndian.PutUint32(desc[12:16], v) }

func (d *Device) txDescOff(i int) int { return i * descStride }
func (d *Device) rxDescOff(i int) int { return txCount*descStride + i*descStride }
func (d *Device) txBufOff(i int) int  { return txCount*descStride + rxCount*descStride + i*bufSize }
func (d *Device) rxBufOff(i int) int {
	return txCount*descStride + rxCount*descStride + txCount*bufSize + i*bufSize
}

func (d *Device) dmaClean(off, size int) {
	if d.DMA.Clean != nil {
		d.DMA.Clean(uintptr(unsafe.Pointer(&d.dma[off])), uintptr(size))
	}
}

func (d *Device) dmaInvalidate(off, size int) {
	if d.DMA.Invalidate != nil {
		d.DMA.Invalidate(uintptr(unsafe.Pointer(&d.dma[off])), uintptr(size))
	}
}
