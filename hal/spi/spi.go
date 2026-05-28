// Package spi provides ASPEED SPI flash controller helpers.
package spi

import (
	"errors"

	"github.com/kyanitecomputer/aspeed-go/reg"
)

const (
	AST2700FMCBase   = 0x14000000
	AST2700FMCWindow = 0x100000000

	offCEType = 0x00
	offCE0Ctl = 0x10
	offSeg0   = 0x30

	ctrlUserMode   = 0x3
	ctrlStopActive = 1 << 2
	ctrlClockMask  = 0x0f000f00
)

var ErrInvalidCS = errors.New("spi: invalid chip select")

// Controller is an ASPEED SPI/FMC controller.
type Controller struct {
	Base       uint32
	WindowBase uint64
	MaxCS      int
	Clock      uint32
	devices    [5]Device
}

// Device is one chip-select on an ASPEED SPI/FMC controller.
type Device struct {
	controller *Controller
	CS         int
	WindowBase uint64
	WindowSize int64
	ctrl       uint32
}

// Init initializes the controller in user mode.
func (c *Controller) Init() error {
	if c.MaxCS <= 0 || c.MaxCS > len(c.devices) {
		return ErrInvalidCS
	}
	if c.Clock == 0 {
		c.Clock = 0x0400
	}
	reg.SetBits32(uintptr(c.Base+offCEType), uint32((1<<c.MaxCS)-1)<<16)
	for cs := 0; cs < c.MaxCS; cs++ {
		ctl := ctrlStopActive | ctrlUserMode | (c.Clock & ctrlClockMask)
		reg.Write(c.Base+uint32(offCE0Ctl+cs*4), ctl)
		c.devices[cs] = Device{controller: c, CS: cs, WindowBase: c.windowBase(cs), WindowSize: c.windowSize(cs), ctrl: ctl}
	}
	return nil
}

// Device returns a chip-select device.
func (c *Controller) Device(cs int) (*Device, error) {
	if cs < 0 || cs >= c.MaxCS {
		return nil, ErrInvalidCS
	}
	return &c.devices[cs], nil
}

// TxRx transfers bytes in user mode.
func (d *Device) TxRx(tx []byte, rx []byte) {
	d.activate()
	for _, b := range tx {
		reg.Write8(uintptr(d.WindowBase), b)
	}
	for i := range rx {
		rx[i] = reg.Read8(uintptr(d.WindowBase))
	}
	d.deactivate()
}

func (d *Device) activate() {
	reg.Write(d.controller.Base+uint32(offCE0Ctl+d.CS*4), d.ctrl)
	reg.Write(d.controller.Base+uint32(offCE0Ctl+d.CS*4), d.ctrl&^ctrlStopActive)
}

func (d *Device) deactivate() {
	reg.Write(d.controller.Base+uint32(offCE0Ctl+d.CS*4), d.ctrl|ctrlStopActive)
}

func (c *Controller) windowBase(cs int) uint64 {
	if cs == 0 {
		return c.WindowBase
	}
	return c.devices[cs-1].WindowBase + uint64(c.devices[cs-1].WindowSize)
}

func (c *Controller) windowSize(cs int) int64 {
	seg := reg.Read(c.Base + uint32(offSeg0+cs*4))
	start := uint64(seg&0x0000ffff) << 16
	end := uint64(seg & 0xffff0000)
	if end <= start {
		return 0x10000
	}
	return int64(end - start + 0x10000)
}
