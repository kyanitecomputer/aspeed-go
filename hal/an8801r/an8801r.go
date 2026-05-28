// Package an8801r provides Airoha AN8801R Ethernet PHY initialization.
package an8801r

import (
	"errors"
	"fmt"
	"time"
)

const (
	PHYID = 0xc0ff0421

	regBMCR   = 0x00
	regBMSR   = 0x01
	regID1    = 0x02
	regID2    = 0x03
	regANAR   = 0x04
	regANLPAR = 0x05
	regGBCR   = 0x09
	regGBSR   = 0x0a
	regPage   = 0x1f

	bmcrReset     = 1 << 15
	bmcrANEnable  = 1 << 12
	bmcrANRestart = 1 << 9
	bmsrLink      = 1 << 2
	bmsrANDone    = 1 << 5

	anar10Half   = 1 << 5
	anar10Full   = 1 << 6
	anar100Half  = 1 << 7
	anar100Full  = 1 << 8
	gbcr1000Half = 1 << 8
	gbcr1000Full = 1 << 9
	gbsr1000Half = 1 << 10
	gbsr1000Full = 1 << 11

	ephyAddr = 0x11000000
	cl22Flag = 0x00800000

	ledBCR        = 0x021
	ledBCRExtCtrl = 1 << 15
	ledBCRClkEn   = 1 << 3
	ledOnDUR      = 0x022
	ledBlkDUR     = 0x023
	ledOnEN       = 1 << 15

	ledBlkDur128M = 2

	ledOnEvtLink1000M = 1 << 0
	ledOnEvtLink100M  = 1 << 1
	ledOnEvtLink10M   = 1 << 2

	ledBlkEvt1000MTX = 1 << 0
	ledBlkEvt1000MRX = 1 << 1
	ledBlkEvt100MTX  = 1 << 2
	ledBlkEvt100MRX  = 1 << 3
	ledBlkEvt10MTX   = 1 << 4
	ledBlkEvt10MRX   = 1 << 5
)

var (
	ErrBadID   = errors.New("an8801r: unexpected phy id")
	ErrNoLink  = errors.New("an8801r: link down")
	ErrTimeout = errors.New("an8801r: timeout")
)

// MDIOBus is the MDIO access needed by Device.
type MDIOBus interface {
	Read(phyAddr, devAddr uint8, regAddr uint16) (uint16, error)
	Write(phyAddr, devAddr uint8, regAddr, value uint16) error
}

// Link describes the negotiated PHY link mode.
type Link struct {
	SpeedMbps  int
	FullDuplex bool
}

// Device is an AN8801R PHY on an MDIO bus.
type Device struct {
	MDIO MDIOBus
	Addr uint8

	// rgmiiTxDelay/rgmiiRxDelay record the values written to the RGMII
	// internal-delay registers. Those analog ("buck") registers are not
	// reliably read-backable, so these cached writes are the source of truth
	// when reporting the configured delays.
	rgmiiTxDelay uint32
	rgmiiRxDelay uint32
}

// ID reads the PHY identifier.
func (d *Device) ID() (uint32, error) {
	id1, err := d.read(regID1)
	if err != nil {
		return 0, err
	}
	id2, err := d.read(regID2)
	if err != nil {
		return 0, err
	}
	return uint32(id1)<<16 | uint32(id2), nil
}

// InitRGMIIID initializes the PHY for RGMII with internal TX/RX delays.
func (d *Device) InitRGMIIID() error {
	id, err := d.ID()
	if err != nil {
		return fmt.Errorf("read id: %w", err)
	}
	if id != PHYID {
		return fmt.Errorf("%w: %#08x", ErrBadID, id)
	}
	if err := d.reset(); err != nil {
		return err
	}
	if err := d.initAnalog(); err != nil {
		return err
	}
	if err := d.cl45Write(0x07, 0x003c, 0); err != nil {
		return fmt.Errorf("disable eee advertisement: %w", err)
	}
	if err := d.configureDelays(); err != nil {
		return err
	}
	if err := d.RestartAutoNegotiation(); err != nil {
		return err
	}
	return nil
}

// RestartAutoNegotiation restarts PHY link auto-negotiation.
func (d *Device) RestartAutoNegotiation() error {
	ctl, err := d.read(regBMCR)
	if err != nil {
		return err
	}
	return d.write(regBMCR, ctl|bmcrANEnable|bmcrANRestart)
}

// WaitLink waits for link and returns the negotiated mode.
func (d *Device) WaitLink(timeout time.Duration) (Link, error) {
	deadline := time.Now().Add(timeout)
	_, _ = d.read(regBMSR)
	for time.Now().Before(deadline) {
		status, err := d.read(regBMSR)
		if err != nil {
			return Link{}, err
		}
		if status&bmsrLink != 0 && status&bmsrANDone != 0 {
			return d.negotiatedLink()
		}
		time.Sleep(100 * time.Millisecond)
	}
	return Link{}, ErrTimeout
}

// rgmiiSpeedReg selects the PHY's RGMII speed mode. The vendor driver sets
// bit0 for a 1000M link and clears it otherwise once the negotiated speed is
// known; without it the PHY's gigabit RGMII path is misconfigured and large TX
// frames are corrupted while short frames still pass.
const rgmiiSpeedReg = 0x10005054

// ApplyLinkSpeed programs the PHY's speed-dependent RGMII configuration for the
// negotiated link. Call this after WaitLink reports the link is up.
func (d *Device) ApplyLinkSpeed(link Link) error {
	set := uint32(0)
	if link.SpeedMbps == 1000 {
		set = 1
	}
	return d.buckModify(rgmiiSpeedReg, 1, set)
}

func (d *Device) reset() error {
	if err := d.write(regBMCR, bmcrReset); err != nil {
		return err
	}
	for i := 0; i < 50; i++ {
		time.Sleep(10 * time.Millisecond)
		ctl, err := d.read(regBMCR)
		if err == nil && ctl&bmcrReset == 0 {
			return nil
		}
	}
	return ErrTimeout
}

func (d *Device) initAnalog() error {
	writes := []struct{ addr, value uint32 }{
		{0x11f808d0, 0x180},
		{0x1021c004, 0x1},
		{0x10270004, 0x3f},
		{0x10270104, 0xff},
		{0x10270204, 0xff},
		{0x100001a4, 0x3},
	}
	if err := d.cl45Write(0x1f, 0x0600, 0x001e); err != nil {
		return err
	}
	if err := d.cl45Write(0x1f, 0x0601, 0x0002); err != nil {
		return err
	}
	if err := d.write(regPage, 1); err != nil {
		return err
	}
	if err := d.write(0x14, 0x3a14); err != nil {
		return err
	}
	if err := d.write(regPage, 0); err != nil {
		return err
	}
	for _, w := range writes {
		if err := d.buckWrite(w.addr, w.value); err != nil {
			return err
		}
	}
	for _, w := range []struct{ dev, reg, val uint16 }{{0x1e, 0x13, 0x4040}, {0x1e, 0xd8, 0x1010}, {0x1e, 0xd9, 0x0100}, {0x1e, 0xda, 0x0100}} {
		if err := d.cl45Write(w.dev, w.reg, w.val); err != nil {
			return err
		}
	}
	return nil
}

// RGMII internal-delay registers on the AN8801R analog ("buck") bus. These are
// the vendor's calibrated values (matching the original Linux/u-boot driver).
const (
	rgmiiTxDelayReg = 0x1021c024
	rgmiiTxDelayVal = 0x01000004
	rgmiiRxDelayReg = 0x1021c02c
	rgmiiRxDelayVal = 0x01000010
)

func (d *Device) configureDelays() error {
	if err := d.buckWrite(rgmiiTxDelayReg, rgmiiTxDelayVal); err != nil {
		return fmt.Errorf("tx delay: %w", err)
	}
	if err := d.buckWrite(rgmiiRxDelayReg, rgmiiRxDelayVal); err != nil {
		return fmt.Errorf("rx delay: %w", err)
	}
	d.rgmiiTxDelay = rgmiiTxDelayVal
	d.rgmiiRxDelay = rgmiiRxDelayVal
	return nil
}

// ConfiguredRGMIIDelays returns the RGMII TX/RX internal-delay register values
// last written by configureDelays. The registers are write-only, so these are
// the values actually programmed (not a hardware read-back).
func (d *Device) ConfiguredRGMIIDelays() (tx, rx uint32) {
	return d.rgmiiTxDelay, d.rgmiiRxDelay
}

func (d *Device) negotiatedLink() (Link, error) {
	gbcr, err := d.read(regGBCR)
	if err != nil {
		return Link{}, err
	}
	gbsr, err := d.read(regGBSR)
	if err != nil {
		return Link{}, err
	}
	common1000 := gbcr & (gbsr >> 2)
	switch {
	case common1000&gbcr1000Full != 0:
		return Link{SpeedMbps: 1000, FullDuplex: true}, nil
	case common1000&gbcr1000Half != 0:
		return Link{SpeedMbps: 1000}, nil
	}
	anar, err := d.read(regANAR)
	if err != nil {
		return Link{}, err
	}
	anlpar, err := d.read(regANLPAR)
	if err != nil {
		return Link{}, err
	}
	common := anar & anlpar
	switch {
	case common&anar100Full != 0:
		return Link{SpeedMbps: 100, FullDuplex: true}, nil
	case common&anar100Half != 0:
		return Link{SpeedMbps: 100}, nil
	case common&anar10Full != 0:
		return Link{SpeedMbps: 10, FullDuplex: true}, nil
	case common&anar10Half != 0:
		return Link{SpeedMbps: 10}, nil
	default:
		return Link{}, ErrNoLink
	}
}

// InitLEDs configures the three AN8801R PHY LEDs to match the vendor
// Linux driver defaults:
//
//   - LED0 (GPIO1): link on any speed, no blink
//   - LED1 (GPIO2): no steady, blink on all TX/RX activity
//   - LED2 (GPIO3): link 100M/10M, blink on 100M/10M TX/RX
func (d *Device) InitLEDs() error {
	type ledCfg struct {
		gpio   uint8
		onEvt  uint16
		blkEvt uint16
	}
	leds := [3]ledCfg{
		{1, ledOnEvtLink1000M | ledOnEvtLink100M | ledOnEvtLink10M, 0},
		{2, 0, ledBlkEvt1000MTX | ledBlkEvt1000MRX | ledBlkEvt100MTX | ledBlkEvt100MRX | ledBlkEvt10MTX | ledBlkEvt10MRX},
		{3, ledOnEvtLink100M | ledOnEvtLink10M, ledBlkEvt100MTX | ledBlkEvt100MRX | ledBlkEvt10MTX | ledBlkEvt10MRX},
	}
	blinkDur := uint16(1024 << ledBlkDur128M)
	if err := d.cl45Write(0x1f, ledBlkDUR, blinkDur); err != nil {
		return err
	}
	if err := d.cl45Write(0x1f, ledOnDUR, blinkDur/2); err != nil {
		return err
	}
	if err := d.cl45Write(0x1f, ledBCR, ledBCRExtCtrl|ledBCRClkEn); err != nil {
		return err
	}
	for i, led := range leds {
		onCtrl := uint16(0x024) + uint16(i)*2
		blkCtrl := uint16(0x025) + uint16(i)*2
		if err := d.cl45Write(0x1f, onCtrl, ledOnEN|led.onEvt); err != nil {
			return err
		}
		if err := d.cl45Write(0x1f, blkCtrl, led.blkEvt); err != nil {
			return err
		}
		if err := d.buckModify(0x10000054, 1<<led.gpio, 1<<led.gpio); err != nil {
			return err
		}
		sel := uint32(i) << (uint32(led.gpio) * 3)
		if err := d.buckModify(0x10000058, sel, sel); err != nil {
			return err
		}
		if err := d.buckModify(0x10000070, 1<<led.gpio, 0); err != nil {
			return err
		}
	}
	return nil
}

func (d *Device) cl45Write(devad, regAddr, value uint16) error {
	addr := ephyAddr | cl22Flag | uint32(devad)<<18 | uint32(regAddr)<<2
	return d.buckWrite(addr, uint32(value))
}

func (d *Device) buckWrite(addr, value uint32) error {
	if err := d.write(regPage, 4); err != nil {
		return err
	}
	if err := d.write(0x10, 0); err != nil {
		return err
	}
	if err := d.write(0x11, uint16(addr>>16)); err != nil {
		return err
	}
	if err := d.write(0x12, uint16(addr)); err != nil {
		return err
	}
	if err := d.write(0x13, uint16(value>>16)); err != nil {
		return err
	}
	if err := d.write(0x14, uint16(value)); err != nil {
		return err
	}
	return d.write(regPage, 0)
}

func (d *Device) buckRead(addr uint32) (uint32, error) {
	if err := d.write(regPage, 4); err != nil {
		return 0, err
	}
	if err := d.write(0x10, 0); err != nil {
		return 0, err
	}
	if err := d.write(0x15, uint16(addr>>16)); err != nil {
		return 0, err
	}
	if err := d.write(0x16, uint16(addr)); err != nil {
		return 0, err
	}
	hi, err := d.read(0x17)
	if err != nil {
		return 0, err
	}
	lo, err := d.read(0x18)
	if err != nil {
		return 0, err
	}
	if err := d.write(regPage, 0); err != nil {
		return 0, err
	}
	return uint32(hi)<<16 | uint32(lo), nil
}

func (d *Device) buckModify(addr, mask, set uint32) error {
	val, err := d.buckRead(addr)
	if err != nil {
		return err
	}
	return d.buckWrite(addr, (val&^mask)|set)
}

func (d *Device) read(regAddr uint16) (uint16, error) {
	return d.MDIO.Read(d.Addr, 0, regAddr)
}

func (d *Device) write(regAddr, value uint16) error {
	return d.MDIO.Write(d.Addr, 0, regAddr, value)
}
