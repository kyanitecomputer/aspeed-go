// Package vhub provides an ASPEED AST2700 USB "virtual hub" (vHub) gadget
// controller bring-up driver.
//
// The AST2700 USB device controller presents itself to the host as a USB-2.0
// hub with up to 7 independent downstream gadget ports plus a shared pool of
// generic endpoints. This package implements the controller-level bring-up:
// SCU clock enable / reset deassert / port function mux, PHY enable, root-hub
// soft reset, and the upstream (host-visible) pull-up "connect". It exposes a
// read-only status/interrupt view for diagnostics and, once the endpoint/DMA
// and descriptor layers land, will back the full gadget stack.
//
// The register model matches vhub.h in the Linux aspeed-vhub driver and the
// u-boot bootusb bring-up (identical IP across Linux/u-boot/Zephyr). See
// aspeed-data/data/registers/usb_vhub_ast2700_v1.yaml for the authoritative
// register definitions.
//
// This is a bring-up / diagnostic driver: EP0 enumeration, generic-endpoint
// DMA, and IRQ-driven operation are not yet implemented (polled only).
//
// Intended for GOOS=tamago GOARCH=arm64 on the CA35.
package vhub

import (
	"time"

	"github.com/kyanitecomputer/aspeed-go/reg"
)

// Global / root-hub register offsets, relative to the controller base.
const (
	regCTRL       = 0x00 // Root function control & status.
	regCONF       = 0x04 // Root configuration (vHub's own USB address).
	regIER        = 0x08 // Interrupt enable.
	regISR        = 0x0C // Interrupt status (write 1 to acknowledge).
	regEPACKIER   = 0x10 // Endpoint-pool ACK interrupt enable.
	regEPNACKIER  = 0x14 // Endpoint-pool NACK interrupt enable.
	regEPACKISR   = 0x18 // Endpoint-pool ACK interrupt status.
	regEPNACKISR  = 0x1C // Endpoint-pool NACK interrupt status.
	regSWRESET    = 0x20 // Device controller soft reset.
	regUSBSTS     = 0x24 // USB status (speed + frame number).
	regEPTOGGLE   = 0x28 // Endpoint-pool data-toggle set/clear.
	regISOFAILACC = 0x2C // Isochronous transaction fail accumulator.
	regEP0CTRL    = 0x30 // Hub EP0 control/status.
	regEP0DATA    = 0x34 // Hub EP0 data buffer DMA base.
	regEP1CTRL    = 0x38 // Hub EP1 (port status-change) control.
	regEP1STSCHG  = 0x3C // Hub EP1 status-change bitmap.
	regSETUP0     = 0x80 // Root/hub SETUP buffer, bytes 0..3.
	regSETUP1     = 0x84 // Root/hub SETUP buffer, bytes 4..7.
	regPHYCTRL    = 0x800
)

// CTRL (0x00) bits.
const (
	ctrlUpstreamConnect  = 1 << 0
	ctrlFullSpeedOnly    = 1 << 1
	ctrlClkStopSuspend   = 1 << 2
	ctrlAutoRemoteWakeup = 1 << 3
	ctrlSplitIn          = 1 << 16
	ctrlISORspCtrl       = 1 << 17
	ctrlLongDesc         = 1 << 18
	ctrlEnlargeFIFO      = 1 << 21
	ctrlPHYResetDis      = 1 << 11
	ctrlPHYClk           = 1 << 31
)

// IER/ISR (0x08/0x0C) bits.
const (
	irqHubEP0Setup    = 1 << 0
	irqHubEP0OutAck   = 1 << 1
	irqHubEP0InAck    = 1 << 3
	irqHubEP1InAck    = 1 << 5
	irqBusReset       = 1 << 6
	irqBusSuspend     = 1 << 7
	irqBusResume      = 1 << 8
	irqDevice1        = 1 << 9
	irqEPPoolAckStall = 1 << 16
	irqEPPoolNak      = 1 << 17

	// irqACKAll acknowledges every hub/bus interrupt source.
	irqACKAll = 0x1ff
)

// SW_RESET (0x20) bits.
const (
	swResetRootHub       = 1 << 0
	swResetDMAController  = 1 << 8
	swResetEPPool        = 1 << 9
)

// USBSTS (0x24) bits.
const usbstsHispeed = 1 << 27

// PHY_CTRL (0x800) bits.
const (
	phyCPUDieSRAMEn   = 1 << 4  // vHubA/vHubB (CPU die) SRAM access enable.
	phyIODieAHBAddr34 = 1 << 5  // vHubC/vHubD (IO die) AHB master addr bit 34.
	phyIODieSRAMEn    = 1 << 10 // vHubC/vHubD (IO die) SRAM access enable.
	phyFIFOForceRetry = 1 << 13 // SoC0 vHub0/vHubB0 txfifo-retry quirk.
)

// SCU register offsets (shared by SCU0 @ 0x12c0_2000 and SCU1 @ 0x14c0_2000).
// Resets for USB clock IDs >= 32 live in RST_CTRL2 (0x220 assert / 0x224 clear);
// clock gates are cleared (enabled) by writing to CLK_STOP + 0x04.
const (
	scuProtect     = 0x000
	scuRstCtrl2    = 0x220
	scuRstCtrl2Clr = 0x224
	scuClkStop     = 0x240
	scuClkStopClr  = 0x244

	scuProtectKey = 0x1688a8a8
)

// Port describes one vHub controller instance and its SCU wiring. Only the
// die0 (CPU-die, GIC-SPI) ports are predefined; die1 (IO-die) ports are
// deferred pending SCU1 reset/clock-bit verification and the INTC1_4 driver.
type Port struct {
	Name    string
	Base    uint32 // Controller register base.
	SCUBase uint32 // SCU controlling this port (SCU0 for die0).
	IRQ     int    // GIC SPI number (die0) for reference.

	ClockBit uint32 // CLK_STOP bit; enabled by writing to CLK_STOP+0x04.
	ResetBit uint32 // RST_CTRL2 (0x220) bit; asserted at 0x220, released at 0x224.

	FuncMux  uint32 // SCU port-function mux register offset (0 = skip).
	FuncMask uint32 // Mux field mask.
	FuncBits uint32 // Mux value selecting device (gadget) mode.

	IODie            bool // true for die1 (IO-die) controllers.
	TXFIFORetryQuirk bool // set PHY FIFO_FORCE_RETRY (SoC0 vHub0/vHubB0).
}

// Predefined DC-SCM-enabled die0 gadget ports.
//
// Port-A/B function mux (SCU0 0x410): device mode selects bits[25:24]=0b10
// (USB2AD) for port A and bits[29:28]=0b10 (USB2BD) for port B, per the u-boot
// pinctrl group table.
var (
	// VHubA0 is the primary DC-SCM gadget port (port A, device mux already set
	// by the board DTS). clk PORTAUSB2CLK, rst PORTA_VHUB_EHCI.
	VHubA0 = Port{
		Name: "vhuba0", Base: 0x12060000, SCUBase: 0x12c02000, IRQ: 33,
		ClockBit: 1 << 14, ResetBit: 1 << 6,
		FuncMux: 0x410, FuncMask: 0x3 << 24, FuncBits: 0x2 << 24,
		TXFIFORetryQuirk: true,
	}

	// VHubB0 is the secondary DC-SCM gadget port (port B).
	// clk PORTBUSB2CLK, rst PORTB_VHUB_EHCI.
	VHubB0 = Port{
		Name: "vhubb0", Base: 0x12062000, SCUBase: 0x12c02000, IRQ: 37,
		ClockBit: 1 << 7, ResetBit: 1 << 7,
		FuncMux: 0x410, FuncMask: 0x3 << 28, FuncBits: 0x2 << 28,
		TXFIFORetryQuirk: true,
	}
)

// Controller is a single AST2700 vHub gadget controller.
type Controller struct {
	port Port
}

// New returns a controller for the given port.
func New(p Port) *Controller { return &Controller{port: p} }

// Port returns the controller's port descriptor.
func (c *Controller) Port() Port { return c.port }

func (c *Controller) r(off uint32) uint32     { return reg.Read(c.port.Base + off) }
func (c *Controller) w(off uint32, v uint32)  { reg.Write(c.port.Base + off, v) }
func (c *Controller) scuUnlock()              { reg.Write(c.port.SCUBase+scuProtect, scuProtectKey) }
func (c *Controller) scuLock()                { reg.Write(c.port.SCUBase+scuProtect, 1) }

// EnableClockReset applies the SCU-level bring-up for the port: select the
// device (gadget) function mux, assert reset, enable the port clock, wait for
// the PLL to lock, then deassert reset. Mirrors u-boot usb_pinctrl +
// usb_clk_enable_reset.
func (c *Controller) EnableClockReset() {
	p := c.port
	c.scuUnlock()
	if p.FuncMux != 0 {
		reg.MaskWrite32(uintptr(p.SCUBase+p.FuncMux), p.FuncMask, p.FuncBits)
	}
	reg.Write(p.SCUBase+scuRstCtrl2, p.ResetBit)   // assert reset
	reg.Write(p.SCUBase+scuClkStopClr, p.ClockBit) // enable clock (clear stop)
	c.scuLock()

	time.Sleep(10 * time.Millisecond) // wait PLL lock

	c.scuUnlock()
	reg.Write(p.SCUBase+scuRstCtrl2Clr, p.ResetBit) // deassert reset
	c.scuLock()

	time.Sleep(time.Millisecond)
}

// initHW enables SRAM access, brings the PHY up, and soft-resets the root hub.
// Mirrors u-boot usb_init (with the Linux txfifo-retry quirk for the affected
// parts). It leaves the controller ready but NOT yet connected to the host.
func (c *Controller) initHW() {
	p := c.port

	// Enable SRAM access (die-specific bits in PHY_CTRL).
	v := c.r(regPHYCTRL)
	if p.IODie {
		v |= phyIODieSRAMEn | phyIODieAHBAddr34
	} else {
		v |= phyCPUDieSRAMEn
	}
	c.w(regPHYCTRL, v)

	// PHY clock on, hold internal PHY reset off.
	c.w(regCTRL, ctrlPHYClk|ctrlPHYResetDis)

	// Soft-reset the root hub.
	c.w(regSWRESET, swResetRootHub)
	time.Sleep(time.Microsecond)
	c.w(regSWRESET, 0)

	if p.TXFIFORetryQuirk {
		c.w(regPHYCTRL, c.r(regPHYCTRL)|phyFIFOForceRetry)
	}
}

// Init performs the full controller bring-up short of the host-visible attach:
// SCU clock/reset/mux, PHY enable, and root-hub soft reset. Call Connect
// afterwards to make the vHub visible to the host.
func (c *Controller) Init() {
	c.EnableClockReset()
	c.initHW()
}

// Connect asserts the upstream D+ pull-up so the host detects and enumerates
// the vHub, and enables automatic remote wakeup.
func (c *Controller) Connect() {
	c.w(regCTRL, c.r(regCTRL)|ctrlUpstreamConnect|ctrlAutoRemoteWakeup)
}

// Disconnect clears the upstream pull-up, detaching the vHub from the host.
func (c *Controller) Disconnect() {
	c.w(regCTRL, c.r(regCTRL)&^ctrlUpstreamConnect)
}

// ISR returns the current interrupt-status bits.
func (c *Controller) ISR() uint32 { return c.r(regISR) }

// AckISR acknowledges (write-1-to-clear) the given interrupt-status bits.
func (c *Controller) AckISR(mask uint32) { c.w(regISR, mask) }

// PollBusEvents polls the interrupt-status register for the given duration,
// acknowledging and accumulating every bit seen. It returns the OR of all
// observed ISR bits — useful to confirm the host issued a bus reset / SETUP
// after Connect without needing the interrupt controller.
func (c *Controller) PollBusEvents(d time.Duration) uint32 {
	var seen uint32
	deadline := time.Now().Add(d)
	for time.Now().Before(deadline) {
		if isr := c.r(regISR); isr != 0 {
			seen |= isr
			c.w(regISR, isr) // acknowledge
		}
		time.Sleep(time.Millisecond)
	}
	return seen
}

// Status is a snapshot of the controller and its SCU wiring for diagnostics.
type Status struct {
	Ctrl    uint32
	Conf    uint32
	IER     uint32
	ISR     uint32
	USBSTS  uint32
	EP0Ctrl uint32
	EP1Ctrl uint32
	PHYCtrl uint32

	SCUClkStop uint32
	SCUReset   uint32
	SCUFuncMux uint32
}

// Status reads the controller and SCU registers into a snapshot.
func (c *Controller) Status() Status {
	p := c.port
	s := Status{
		Ctrl:       c.r(regCTRL),
		Conf:       c.r(regCONF),
		IER:        c.r(regIER),
		ISR:        c.r(regISR),
		USBSTS:     c.r(regUSBSTS),
		EP0Ctrl:    c.r(regEP0CTRL),
		EP1Ctrl:    c.r(regEP1CTRL),
		PHYCtrl:    c.r(regPHYCTRL),
		SCUClkStop: reg.Read(p.SCUBase + scuClkStop),
		SCUReset:   reg.Read(p.SCUBase + scuRstCtrl2),
	}
	if p.FuncMux != 0 {
		s.SCUFuncMux = reg.Read(p.SCUBase + p.FuncMux)
	}
	return s
}

// Connected reports whether the upstream pull-up is asserted.
func (s Status) Connected() bool { return s.Ctrl&ctrlUpstreamConnect != 0 }

// PHYUp reports whether the PHY clock is enabled and its reset held off.
func (s Status) PHYUp() bool { return s.Ctrl&ctrlPHYClk != 0 && s.Ctrl&ctrlPHYResetDis != 0 }

// HighSpeed reports whether the upstream bus enumerated at high speed.
func (s Status) HighSpeed() bool { return s.USBSTS&usbstsHispeed != 0 }

// FrameNumber returns the current USB (micro)frame number.
func (s Status) FrameNumber() uint32 { return (s.USBSTS >> 16) & 0x7ff }

// ClockRunning reports whether the port clock gate is enabled (stop bit clear).
func (s Status) ClockRunning(p Port) bool { return s.SCUClkStop&p.ClockBit == 0 }

// InReset reports whether the port reset is currently asserted.
func (s Status) InReset(p Port) bool { return s.SCUReset&p.ResetBit != 0 }

// Event describes one decodable interrupt-status bit for diagnostics.
type Event struct {
	Bit  uint32
	Name string
}

// DecodeEvents returns the named interrupt sources present in an ISR value.
func DecodeEvents(isr uint32) []Event {
	all := []Event{
		{irqHubEP0Setup, "HUB_EP0_SETUP"},
		{irqHubEP0OutAck, "HUB_EP0_OUT_ACK"},
		{irqHubEP0InAck, "HUB_EP0_IN_ACK"},
		{irqHubEP1InAck, "HUB_EP1_IN_ACK"},
		{irqBusReset, "BUS_RESET"},
		{irqBusSuspend, "BUS_SUSPEND"},
		{irqBusResume, "BUS_RESUME"},
		{irqDevice1, "DEVICE1"},
		{irqEPPoolAckStall, "EP_POOL_ACK_STALL"},
		{irqEPPoolNak, "EP_POOL_NAK"},
	}
	out := make([]Event, 0, len(all))
	for _, e := range all {
		if isr&e.Bit != 0 {
			out = append(out, e)
		}
	}
	return out
}
