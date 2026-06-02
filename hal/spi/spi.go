// Package spi provides ASPEED SPI flash controller helpers.
package spi

import (
	"errors"

	"github.com/kyanitecomputer/aspeed-go/reg"
)

const (
	AST2700FMCBase   = 0x14000000
	AST2700FMCWindow = 0x100000000

	// Controller register offsets (spi_ast2700_v1 / fmc_v1 block).
	offFlashConfig = 0x000 // CE_WRITE_ENABLE bits [19:16], one per CE
	offCE0Ctl      = 0x010 // CE_CTRL_N[0..3] at 0x010,0x014,0x018,0x01C
	offSeg0        = 0x030 // CE_ADDR_RANGE[0..3] at 0x030,0x034,0x038,0x03C
	offMisc        = 0x054 // MISC_CTRL / SAFS mode-select; must be 0 for user PIO

	ceWriteEnableShift = 16

	// CE_CTRL_N bit fields.
	cmdModeMask    = 0x3
	cmdModeUser    = 0x3    // user command mode
	ctrlStopActive = 1 << 2 // CE de-asserted while set
	ioModeMask     = 0xf << 28
)

var (
	ErrInvalidCS  = errors.New("spi: invalid chip select")
	ErrBadSegment = errors.New("spi: segment decode failed")
)

// Controller is an ASPEED SPI/FMC controller.
//
// At rest each chip-select is left in the controller's memory-mapped auto-read
// (XIP) mode, so the flash window returns flash contents directly and byte-wise
// reads are fast. Individual commands (RDID, RDSR, WREN, erase, program) are
// issued via Device.Command, which briefly switches the CE into user mode and
// restores auto-read afterwards.
type Controller struct {
	Base       uint32
	WindowBase uint64
	MaxCS      int
	SoC        SoC
	devices    [4]Device
}

// Device is one chip-select on an ASPEED SPI/FMC controller.
type Device struct {
	controller *Controller
	CS         int
	WindowBase uint64
	WindowSize int64
}

// Init prepares the controller for user-mode commands while leaving every CE in
// auto-read mode for the memory-mapped read path.
func (c *Controller) Init() error {
	if c.MaxCS <= 0 || c.MaxCS > len(c.devices) {
		return ErrInvalidCS
	}
	if c.SoC == SoCUnknown {
		c.SoC = AST2700
	}

	// Enable the controller write path for every CE. Without CE_WRITE_ENABLE
	// the controller silently drops user-mode window writes, so opcode/address
	// bytes never clock out and every command reads back as zero.
	reg.SetBits32(uintptr(c.Base+offFlashConfig), uint32((1<<c.MaxCS)-1)<<ceWriteEnableShift)

	// Disable SAFS mode-select so user PIO reaches the flash.
	reg.Write(c.Base+offMisc, 0)

	for cs := 0; cs < c.MaxCS; cs++ {
		startOff, size, ok := SegmentDecode(c.SoC, reg.Read(c.Base+uint32(offSeg0+cs*4)))
		if !ok {
			return ErrBadSegment
		}
		c.devices[cs] = Device{
			controller: c,
			CS:         cs,
			WindowBase: c.WindowBase + uint64(startOff),
			WindowSize: size,
		}
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

// Command issues one user-mode SPI transaction: it clocks out every byte of tx
// (opcode, address, and any write data) and then clocks in len(rx) reply bytes.
// The CE is switched into user mode for the transfer and restored to its prior
// (auto-read) configuration afterwards, so the memory-mapped window keeps
// working between commands.
//
// The receive loop is a plain read with no dummy writes: on the AST2700 FMC a
// write-then-read receive discipline clocks an extra byte and returns shifted
// data.
func (d *Device) Command(tx, rx []byte) {
	ctlAddr := d.controller.Base + uint32(offCE0Ctl+d.CS*4)
	saved := reg.Read(ctlAddr)
	cu := (saved &^ (ioModeMask | cmdModeMask)) | cmdModeUser

	reg.Write(ctlAddr, cu|ctrlStopActive)  // user mode, CE de-asserted
	reg.Write(ctlAddr, cu&^ctrlStopActive) // assert CE
	_ = reg.Read(ctlAddr)                  // read-back flush

	win := uintptr(d.WindowBase)
	for _, b := range tx {
		reg.Write8(win, b)
	}
	for i := range rx {
		rx[i] = reg.Read8(win)
	}

	reg.Write(ctlAddr, cu|ctrlStopActive) // de-assert CE
	reg.Write(ctlAddr, saved)             // restore auto-read
}

// ReadWindow reads len(dst) bytes from the memory-mapped auto-read window at the
// given device-relative byte offset. It relies on the CE resting in auto-read
// mode (the state Init leaves and Command restores).
func (d *Device) ReadWindow(dst []byte, off int64) {
	base := uintptr(d.WindowBase) + uintptr(off)
	for i := range dst {
		dst[i] = reg.Read8(base + uintptr(i))
	}
}
