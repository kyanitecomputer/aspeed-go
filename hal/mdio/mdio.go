// Package mdio provides an ASPEED MDIO controller driver.
package mdio

import (
	"errors"

	"src.kyanite.computer/aspeed-go/reg"
)

const (
	AST2700MDIO0 = 0x14040000
	AST2700MDIO1 = 0x14040008
	AST2700MDIO2 = 0x14040010

	offCtrl = 0x0
	offData = 0x4

	ctrlFire = 1 << 31
	ctrlC22  = 1 << 28

	opC45Addr  = 0
	opC22Write = 1
	opC22Read  = 2
	opC45Write = 1
	opC45Read  = 3

	dataIdle = 1 << 16
)

var ErrTimeout = errors.New("mdio: timeout")

// Bus is an ASPEED MDIO controller.
type Bus struct {
	Base uint32
}

// Read reads a Clause 22 or Clause 45 PHY register.
func (b *Bus) Read(phyAddr, devAddr uint8, regAddr uint16) (uint16, error) {
	if devAddr == 0 {
		if err := b.op(ctrlC22, opC22Read, phyAddr, uint8(regAddr), 0); err != nil {
			return 0, err
		}
		return b.data()
	}
	if err := b.op(0, opC45Addr, phyAddr, devAddr, regAddr); err != nil {
		return 0, err
	}
	if err := b.op(0, opC45Read, phyAddr, devAddr, 0); err != nil {
		return 0, err
	}
	return b.data()
}

// Write writes a Clause 22 or Clause 45 PHY register.
func (b *Bus) Write(phyAddr, devAddr uint8, regAddr, value uint16) error {
	if devAddr == 0 {
		return b.op(ctrlC22, opC22Write, phyAddr, uint8(regAddr), value)
	}
	if err := b.op(0, opC45Addr, phyAddr, devAddr, regAddr); err != nil {
		return err
	}
	return b.op(0, opC45Write, phyAddr, devAddr, value)
}

func (b *Bus) op(st, op uint32, phyAddr, regAddr uint8, value uint16) error {
	ctrl := uint32(ctrlFire) | st | op<<26 | uint32(phyAddr&0x1f)<<21 | uint32(regAddr&0x1f)<<16 | uint32(value)
	reg.Write(b.Base+offCtrl, ctrl)
	_ = reg.Read(b.Base + offCtrl)
	_ = reg.Read(b.Base + offCtrl)
	for i := 0; i < 1000; i++ {
		if reg.Read(b.Base+offCtrl)&ctrlFire == 0 {
			return nil
		}
	}
	return ErrTimeout
}

func (b *Bus) data() (uint16, error) {
	for i := 0; i < 1000; i++ {
		data := reg.Read(b.Base + offData)
		if data&dataIdle != 0 {
			return uint16(data), nil
		}
	}
	return 0, ErrTimeout
}
