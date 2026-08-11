package aspeedgfx

import (
	"errors"

	"src.kyanite.computer/aspeed-go/reg"
)

// DisplayPort TX AUX/HPD registers (DP base 0x12c0a000). Offsets and the AUX
// transaction sequence follow the ASPEED reference (u-boot cmd/aspeed
// dptest_2700).
const (
	offDPLinkStatus = 0x7c // bit28 = HPD input level (RO)
	offDPIntClear   = 0x40
	offDPIntStatus  = 0x44
	offAUXReqCfg    = 0x88
	offAUXAddrLen   = 0x8c
	offAUXWData0    = 0x90
	offAUXRData0    = 0xa0
	offAUXStatus    = 0xb0

	dpHPDLevel = 1 << 28

	auxIntDone    = 0x4000 // AUX command completed
	auxIntTimeout = 0x8000 // AUX command timed out
	auxIntMask    = auxIntDone | auxIntTimeout

	// I2C-over-AUX commands to the DDC slave (EDID at I2C address 0x50).
	// "M" = middle-of-transaction (MOT set); "EA" = stop after address.
	auxI2CMEAWrite = 0x40010010
	auxI2CMWrite   = 0x40000010
	auxI2CMEARead  = 0x50010010
	auxI2CMRead    = 0x50000010
	auxI2CEARead   = 0x10010010

	auxStatusDefer = 0x20 // I2C defer: retry the transaction

	ddcI2CAddr = 0x50
)

// auxSpinBudget bounds the busy-wait for one AUX transaction so a missing or
// unresponsive sink cannot hang the caller.
const auxSpinBudget = 2_000_000

// auxDeferRetries bounds how many times an I2C-deferred read is retried.
const auxDeferRetries = 64

var (
	// ErrAUXTimeout is returned when an AUX transaction does not complete.
	ErrAUXTimeout = errors.New("aspeedgfx: AUX transaction timeout")
	// ErrNoHPD is returned when EDID is requested with no monitor connected.
	ErrNoHPD = errors.New("aspeedgfx: no display connected (HPD low)")
	// ErrEDIDLength is returned when a partial EDID block is read.
	ErrEDIDLength = errors.New("aspeedgfx: short EDID read")
)

// HPDAsserted reports whether a monitor's hot-plug-detect line is high.
func HPDAsserted() bool {
	return reg.Read(AST2700DPBase+offDPLinkStatus)&dpHPDLevel != 0
}

// auxWrite performs an AUX/I2C-over-AUX write of up to 16 bytes. It returns the
// AUX reply status byte.
func auxWrite(cmd, addr uint32, data []byte) (status byte, err error) {
	if len(data) > 16 {
		return 0, errors.New("aspeedgfx: AUX write too long")
	}
	n := len(data)
	if n < 1 {
		n = 1 // address-only transactions still carry one length unit
	}
	reg.Write(AST2700DPBase+offAUXAddrLen, uint32(n-1)<<24|addr)

	var words [4]uint32
	for i, b := range data {
		words[i/4] |= uint32(b) << (8 * (i % 4))
	}
	for i := 0; i < 4; i++ {
		reg.Write(AST2700DPBase+offAUXWData0+uint32(i)*4, words[i])
	}

	return auxFire(cmd)
}

// auxRead performs an AUX/I2C-over-AUX read of n (1..16) bytes into out. out
// must have length n. It returns the AUX reply status byte.
func auxRead(cmd, addr uint32, out []byte) (status byte, err error) {
	n := len(out)
	if n < 1 || n > 16 {
		return 0, errors.New("aspeedgfx: AUX read length out of range")
	}
	reg.Write(AST2700DPBase+offAUXAddrLen, uint32(n-1)<<24|addr)

	status, err = auxFire(cmd)
	if err != nil {
		return status, err
	}

	var words [4]uint32
	for i := 0; i < (n+3)/4; i++ {
		words[i] = reg.Read(AST2700DPBase + offAUXRData0 + uint32(i)*4)
	}
	for i := 0; i < n; i++ {
		out[i] = byte(words[i/4] >> (8 * (i % 4)))
	}
	return status, nil
}

// auxFire issues an AUX command and waits for completion, returning the reply
// status byte.
func auxFire(cmd uint32) (status byte, err error) {
	reg.Write(AST2700DPBase+offAUXReqCfg, cmd)

	var st uint32
	for spin := 0; spin < auxSpinBudget; spin++ {
		st = reg.Read(AST2700DPBase+offDPIntStatus) & auxIntMask
		if st != 0 {
			break
		}
	}
	if st == 0 {
		return 0, ErrAUXTimeout
	}
	// Clear the latched AUX interrupt status (mirrors dptest: INT_CLEAR |= st>>8).
	reg.Write(AST2700DPBase+offDPIntClear, reg.Read(AST2700DPBase+offDPIntClear)|(st>>8))

	if st&auxIntDone == 0 {
		return 0, ErrAUXTimeout
	}
	return byte(reg.Read(AST2700DPBase+offAUXStatus) & 0xFF), nil
}

// readEDIDBlock reads one 128-byte EDID block at the given byte offset over
// I2C-over-AUX (DDC address 0x50), 16 bytes per AUX transaction.
func readEDIDBlock(offset byte, dst []byte) error {
	// Set the DDC read pointer: address-only write, then the offset byte.
	if _, err := auxWrite(auxI2CMEAWrite, ddcI2CAddr, nil); err != nil {
		return err
	}
	if _, err := auxWrite(auxI2CMWrite, ddcI2CAddr, []byte{offset}); err != nil {
		return err
	}
	// Start the read burst.
	tmp := make([]byte, 1)
	if _, err := auxRead(auxI2CMEARead, ddcI2CAddr, tmp); err != nil {
		return err
	}

	for chunk := 0; chunk < 8; chunk++ {
		buf := make([]byte, 16)
		var status byte
		var err error
		for retry := 0; retry < auxDeferRetries; retry++ {
			status, err = auxRead(auxI2CMRead, ddcI2CAddr, buf)
			if err != nil {
				return err
			}
			if status != auxStatusDefer {
				break
			}
		}
		if status == auxStatusDefer {
			return ErrAUXTimeout
		}
		copy(dst[chunk*16:], buf)
	}

	// Stop the transaction.
	_, err := auxRead(auxI2CEARead, ddcI2CAddr, tmp)
	return err
}

// ReadEDID reads the EDID base block (and one extension block if present) from
// the connected display over DP AUX. It requires HPD to be asserted.
func ReadEDID() ([]byte, error) {
	if !HPDAsserted() {
		return nil, ErrNoHPD
	}

	out := make([]byte, 128)
	if err := readEDIDBlock(0x00, out); err != nil {
		return nil, err
	}
	if out[126] > 0 {
		ext := make([]byte, 128)
		if err := readEDIDBlock(0x80, ext); err != nil {
			// Base block is still usable; return it without the extension.
			return out, nil
		}
		out = append(out, ext...)
	}
	return out, nil
}
