// Package otp provides ASPEED AST2700 OTP controller helpers.
package otp

import (
	"errors"

	"src.kyanite.computer/aspeed-go/reg"
)

const (
	AST2700BootMCUBase = 0x14c07000
	AST2700CPUDieBase  = 0x30c07000

	password     = 0x349fe38a
	cmdRead      = 0x23b1e361
	cmdProg      = 0x23b1e364
	cmdProgMulti = 0x23b1e365

	offKey    = 0x000
	offCmd    = 0x004
	offWData0 = 0x008
	offStatus = 0x018
	offAddr   = 0x01c
	offRData  = 0x020
	offECCEn  = 0x0d4

	statusBusy     = 1
	statusCmdShift = 4
	statusCmdMask  = 0xf << statusCmdShift
	statusPass     = 0
	strap14Addr    = 0x420 + 0x0e
	timeoutLoops   = 10000
)

const (
	ROMStart      = 0x0000
	ROMEnd        = 0x03e0
	RBPStart      = ROMEnd
	RBPEnd        = 0x0400
	ConfStart     = RBPEnd
	ConfEnd       = 0x0420
	StrapStart    = ConfEnd
	StrapEnd      = 0x0430
	StrapExtStart = StrapEnd
	StrapExtEnd   = 0x0440
	UserStart     = StrapExtEnd
	UserEnd       = 0x1000
	SecureStart   = UserEnd
	SecureEnd     = 0x1c00
	CaliptraStart = SecureEnd
	CaliptraEnd   = 0x1f80
	SWPUFStart    = CaliptraEnd
	SWPUFEnd      = 0x1fa0
	HWPUFStart    = SWPUFEnd
	HWPUFEnd      = 0x2000
)

var (
	ErrTimeout       = errors.New("otp: command timeout")
	ErrInvalidLength = errors.New("otp: invalid length")
	ErrCommandFailed = errors.New("otp: command failed")
)

type Controller struct {
	Base       uint32
	ECCEnabled bool
}

func (c *Controller) Init() error {
	reg.Write(c.Base+offKey, password)
	enabled, err := c.readECCStrap()
	if err != nil {
		return err
	}
	c.ECCEnabled = enabled
	return nil
}

func (c *Controller) ReadWord(offset uint32) (uint16, error) {
	c.writeECCMode()
	reg.Write(c.Base+offAddr, offset)
	reg.Write(c.Base+offCmd, cmdRead)
	if err := c.waitComplete(); err != nil {
		return 0, err
	}
	return uint16(reg.Read(c.Base + offRData)), nil
}

func (c *Controller) ReadWords(offset uint32, out []uint16) error {
	for i := range out {
		word, err := c.ReadWord(offset + uint32(i))
		if err != nil {
			return err
		}
		out[i] = word
	}
	return nil
}

func (c *Controller) ReadBytes(offset uint32, out []byte) error {
	for i := 0; i < len(out); i += 2 {
		word, err := c.ReadWord(offset + uint32(i/2))
		if err != nil {
			return err
		}
		out[i] = byte(word)
		if i+1 < len(out) {
			out[i+1] = byte(word >> 8)
		}
	}
	return nil
}

func (c *Controller) ProgramWord(offset uint32, data uint16) error {
	c.writeECCMode()
	reg.Write(c.Base+offAddr, offset)
	reg.Write(c.Base+offWData0, uint32(data))
	reg.Write(c.Base+offCmd, cmdProg)
	return c.waitComplete()
}

func (c *Controller) ProgramWords(offset uint32, data []uint16) error {
	if len(data) == 1 {
		return c.ProgramWord(offset, data[0])
	}
	if len(data)%2 != 0 || len(data) > 8 {
		return ErrInvalidLength
	}
	c.writeECCMode()
	reg.Write(c.Base+offAddr, offset)
	for i := 0; i < len(data); i += 2 {
		reg.Write(c.Base+offWData0+uint32(i*2), uint32(data[i])|uint32(data[i+1])<<16)
	}
	reg.Write(c.Base+offCmd, cmdProgMulti)
	return c.waitComplete()
}

func (c *Controller) readECCStrap() (bool, error) {
	reg.Write(c.Base+offECCEn, 0)
	reg.Write(c.Base+offAddr, strap14Addr)
	reg.Write(c.Base+offCmd, cmdRead)
	if err := c.waitComplete(); err != nil {
		return false, err
	}
	return reg.Read(c.Base+offRData)&1 != 0, nil
}

func (c *Controller) writeECCMode() {
	if c.ECCEnabled {
		reg.Write(c.Base+offECCEn, 1)
		return
	}
	reg.Write(c.Base+offECCEn, 0)
}

func (c *Controller) waitComplete() error {
	for i := 0; i < timeoutLoops; i++ {
		status := reg.Read(c.Base + offStatus)
		if status&statusBusy == 0 {
			if (status&statusCmdMask)>>statusCmdShift == statusPass {
				return nil
			}
			return ErrCommandFailed
		}
	}
	return ErrTimeout
}
