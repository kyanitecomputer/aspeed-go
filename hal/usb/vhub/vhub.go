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

// USB2 PHY analog control window (PHY_CTL_STS_1..4), relative to Port.PHYBase.
// On the AST2700 the per-port USB2 PHY lives inside the vHubA1/vHubB1 register
// block (base 0x12011800 / 0x12021800), gated by the PORTx_VHUB reset — it must
// be brought out of reset and tuned before any vHub device core on that port
// (including the EHCI-companion vHubA0/vHubB0) will clock and accept writes.
const (
	phyCtlSts2 = 0x04 // PHY_CTL_STS_2: reference-clock-rate select.
	phyCtlSts3 = 0x08 // PHY_CTL_STS_3: pre-emphasis current.

	phyCtlSts2ClkRateMask = 0x3 << 26 // [27:26] vHub1 clock rate.
	phyCtlSts2ClkRate60M  = 0x3 << 26 // b'11 = 60 MHz.
	phyCtlSts3PreEmphMask  = 0x3 << 21 // [22:21] pre-emphasis current.
	phyCtlSts3PreEmph2     = 0x2 << 21 // b'10 = setting 2.
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

	// PHYResetBit is the RST_CTRL2 bit for the port's shared USB2 PHY
	// (PORTx_VHUB, distinct from this controller's own ResetBit). It gates the
	// PHY that clocks the vHub device core, so it must be deasserted alongside
	// ResetBit. PHYBase is that PHY's analog control window (PHY_CTL_STS_1..4).
	// Both 0 for controllers that carry their own PHY.
	PHYResetBit uint32
	PHYBase     uint32

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
	// by the board DTS). clk PORTAUSB2CLK, controller rst PORTA_VHUB_EHCI;
	// the shared port-A USB2 PHY is inside vHubA1 (0x12011800), gated by
	// PORTA_VHUB.
	VHubA0 = Port{
		Name: "vhuba0", Base: 0x12060000, SCUBase: 0x12c02000, IRQ: 33,
		ClockBit: 1 << 14, ResetBit: 1 << 6,
		FuncMux: 0x410, FuncMask: 0x3 << 24, FuncBits: 0x2 << 24,
		PHYResetBit: 1 << 0, PHYBase: 0x12011800,
		TXFIFORetryQuirk: true,
	}

	// VHubB0 is the secondary DC-SCM gadget port (port B). clk PORTBUSB2CLK,
	// controller rst PORTB_VHUB_EHCI; shared port-B USB2 PHY inside vHubB1
	// (0x12021800), gated by PORTB_VHUB.
	VHubB0 = Port{
		Name: "vhubb0", Base: 0x12062000, SCUBase: 0x12c02000, IRQ: 37,
		ClockBit: 1 << 7, ResetBit: 1 << 7,
		FuncMux: 0x410, FuncMask: 0x3 << 28, FuncBits: 0x2 << 28,
		PHYResetBit: 1 << 3, PHYBase: 0x12021800,
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

// Step records one observable bring-up action for diagnostics: the register it
// touched, its value before and after the write, and the bits that were
// expected to change. It lets a caller print exactly where a bring-up goes
// wrong (e.g. a clock that stays gated because the SCU is locked, or a function
// mux that doesn't take because the write hit a read-only field).
type Step struct {
	Name    string // human-readable action
	RegName string // register touched ("" for pure delays)
	Addr    uint32 // absolute register address
	Before  uint32 // value read before the write
	After   uint32 // value read back after the write
	WantSet uint32 // bits that After must have SET   (0 = not checked)
	WantClr uint32 // bits that After must have CLEAR (0 = not checked)
	Note    string // extra context
}

// Checked reports whether the step has an expected outcome to verify.
func (s Step) Checked() bool { return s.WantSet != 0 || s.WantClr != 0 }

// OK reports whether the observed After value matches the expectation. Steps
// with no expectation (delays, informational reads) are always OK.
func (s Step) OK() bool {
	if s.WantSet != 0 && s.After&s.WantSet != s.WantSet {
		return false
	}
	if s.WantClr != 0 && s.After&s.WantClr != 0 {
		return false
	}
	return true
}

// InitSteps runs the full controller bring-up short of the host-visible attach
// — SCU device-function mux, reset assert, clock enable, PLL wait, reset
// deassert, PHY SRAM enable, PHY clock on, root-hub soft reset, and the
// txfifo-retry quirk — returning one Step per action with before/after register
// values and expectations. Call Connect afterwards to make the vHub visible to
// the host. This is the single source of truth for the bring-up; Init runs it
// and discards the trace.
func (c *Controller) InitSteps() []Step {
	p := c.port
	steps := make([]Step, 0, 12)
	rd := func(off uint32) uint32 { return reg.Read(p.SCUBase + off) }

	// Keep the SCU unlocked for the whole clock/reset sequence.
	lockBefore := rd(scuProtect)
	c.scuUnlock()
	steps = append(steps, Step{
		Name: "unlock SCU", RegName: "SCU_PROTECT", Addr: p.SCUBase + scuProtect,
		Before: lockBefore, After: rd(scuProtect),
		Note: "write key 0x1688a8a8 (0x000 reads back the silicon revision id)",
	})

	// Select the device (gadget) port function mux.
	if p.FuncMux != 0 {
		before := rd(p.FuncMux)
		reg.MaskWrite32(uintptr(p.SCUBase+p.FuncMux), p.FuncMask, p.FuncBits)
		steps = append(steps, Step{
			Name: "select device (gadget) function mux", RegName: "SCU_USB_MULTI_CTRL",
			Addr: p.SCUBase + p.FuncMux, Before: before, After: rd(p.FuncMux),
			WantSet: p.FuncBits, WantClr: p.FuncMask &^ p.FuncBits,
			Note: "port routed to the vHub device controller",
		})
	}

	// Assert reset.
	{
		before := rd(scuRstCtrl2)
		reg.Write(p.SCUBase+scuRstCtrl2, p.ResetBit)
		steps = append(steps, Step{
			Name: "assert port reset", RegName: "SCU_RST_CTRL2",
			Addr: p.SCUBase + scuRstCtrl2, Before: before, After: rd(scuRstCtrl2),
			WantSet: p.ResetBit, Note: "reset held while the clock spins up",
		})
	}

	// Enable the port clock (clear its stop bit); observe CLK_STOP.
	{
		before := rd(scuClkStop)
		reg.Write(p.SCUBase+scuClkStopClr, p.ClockBit)
		steps = append(steps, Step{
			Name: "enable port clock (clear stop)", RegName: "SCU_CLK_STOP",
			Addr: p.SCUBase + scuClkStop, Before: before, After: rd(scuClkStop),
			WantClr: p.ClockBit, Note: "stop bit must read 0 = clock running",
		})
	}

	time.Sleep(10 * time.Millisecond) // PLL lock
	steps = append(steps, Step{Name: "wait 10ms for PLL lock"})

	// Deassert reset — both this controller AND the shared USB2 PHY reset
	// (PORTx_VHUB). The PHY gates the UTMI clock that feeds the vHub device
	// core; without releasing it the core registers (CTRL, SW_RESET) never
	// clock and silently drop writes.
	{
		mask := p.ResetBit | p.PHYResetBit
		before := rd(scuRstCtrl2)
		reg.Write(p.SCUBase+scuRstCtrl2Clr, mask)
		steps = append(steps, Step{
			Name: "deassert controller + PHY reset", RegName: "SCU_RST_CTRL2",
			Addr: p.SCUBase + scuRstCtrl2, Before: before, After: rd(scuRstCtrl2),
			WantClr: mask, Note: "releases vHub core and the shared USB2 PHY",
		})
	}
	c.scuLock()
	time.Sleep(time.Millisecond)

	// Tune the shared USB2 PHY (inside the vHubx1 block): reference clock rate
	// and pre-emphasis, matching the vendor usb_usb2_init. Must happen after the
	// PHY reset is released and before the core is brought up.
	if p.PHYBase != 0 {
		{
			addr := p.PHYBase + phyCtlSts2
			before := reg.Read(addr)
			reg.MaskWrite32(uintptr(addr), phyCtlSts2ClkRateMask, phyCtlSts2ClkRate60M)
			steps = append(steps, Step{
				Name: "set USB2 PHY clock rate (60MHz)", RegName: "PHY_CTL_STS_2",
				Addr: addr, Before: before, After: reg.Read(addr),
				WantSet: phyCtlSts2ClkRate60M, Note: "PHY [27:26]=b11 (inside vHubx1)",
			})
		}
		{
			addr := p.PHYBase + phyCtlSts3
			before := reg.Read(addr)
			reg.MaskWrite32(uintptr(addr), phyCtlSts3PreEmphMask, phyCtlSts3PreEmph2)
			steps = append(steps, Step{
				Name: "set USB2 PHY pre-emphasis", RegName: "PHY_CTL_STS_3",
				Addr: addr, Before: before, After: reg.Read(addr),
				WantSet: phyCtlSts3PreEmph2, WantClr: phyCtlSts3PreEmphMask &^ phyCtlSts3PreEmph2,
				Note: "PHY [22:21]=b10 (inside vHubx1)",
			})
		}
	}

	// Controller reachability: after clock+reset+PHY, CTRL must be writable.
	// A write-probe is the reliable test (a mere read can return a plausible
	// reset value even when the core is unclocked and dropping writes).
	{
		before := c.r(regCTRL)
		c.w(regCTRL, before|ctrlPHYResetDis)
		after := c.r(regCTRL)
		c.w(regCTRL, before) // restore
		steps = append(steps, Step{
			Name: "probe controller write-ability", RegName: "VHUB_CTRL",
			Addr: p.Base + regCTRL, Before: before, After: after,
			WantSet: ctrlPHYResetDis,
			Note: "toggles PHY_RESET_DIS; if it doesn't stick the core is unclocked (PHY not up)",
		})
	}

	// Enable SRAM access (die-specific bits in PHY_CTRL).
	{
		before := c.r(regPHYCTRL)
		want := uint32(phyCPUDieSRAMEn)
		if p.IODie {
			want = phyIODieSRAMEn | phyIODieAHBAddr34
		}
		c.w(regPHYCTRL, before|want)
		steps = append(steps, Step{
			Name: "enable PHY SRAM access", RegName: "VHUB_PHY_CTRL",
			Addr: p.Base + regPHYCTRL, Before: before, After: c.r(regPHYCTRL),
			WantSet: want, Note: "DMA needs SRAM access enabled",
		})
	}

	// PHY clock on, hold internal PHY reset off.
	{
		before := c.r(regCTRL)
		c.w(regCTRL, ctrlPHYClk|ctrlPHYResetDis)
		steps = append(steps, Step{
			Name: "enable PHY clock, hold PHY reset off", RegName: "VHUB_CTRL",
			Addr: p.Base + regCTRL, Before: before, After: c.r(regCTRL),
			WantSet: ctrlPHYClk | ctrlPHYResetDis, Note: "PHY brought up",
		})
	}

	// Soft-reset the root hub (pulse ROOT_HUB then clear).
	{
		before := c.r(regSWRESET)
		c.w(regSWRESET, swResetRootHub)
		time.Sleep(time.Microsecond)
		c.w(regSWRESET, 0)
		steps = append(steps, Step{
			Name: "soft-reset root hub", RegName: "VHUB_SW_RESET",
			Addr: p.Base + regSWRESET, Before: before, After: c.r(regSWRESET),
			WantClr: swResetRootHub, Note: "pulse then release",
		})
	}

	// txfifo-retry quirk for the affected parts.
	if p.TXFIFORetryQuirk {
		before := c.r(regPHYCTRL)
		c.w(regPHYCTRL, before|phyFIFOForceRetry)
		steps = append(steps, Step{
			Name: "set txfifo-retry quirk", RegName: "VHUB_PHY_CTRL",
			Addr: p.Base + regPHYCTRL, Before: before, After: c.r(regPHYCTRL),
			WantSet: phyFIFOForceRetry, Note: "AST2700 SoC0 vHub0/vHubB0 workaround",
		})
	}

	return steps
}

// Init performs the full controller bring-up short of the host-visible attach:
// SCU clock/reset/mux, PHY enable, and root-hub soft reset. Call Connect
// afterwards to make the vHub visible to the host. Use InitSteps for a
// step-by-step diagnostic trace.
func (c *Controller) Init() { c.InitSteps() }

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

	// Shared USB2 PHY analog control (inside the vHubx1 block), valid when the
	// port has a PHYBase.
	HasPHY     bool
	PHYCtlSts2 uint32
	PHYCtlSts3 uint32
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
	if p.PHYBase != 0 {
		s.HasPHY = true
		s.PHYCtlSts2 = reg.Read(p.PHYBase + phyCtlSts2)
		s.PHYCtlSts3 = reg.Read(p.PHYBase + phyCtlSts3)
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

func filterEvents(all []Event, v uint32) []Event {
	out := make([]Event, 0, len(all))
	for _, e := range all {
		if v&e.Bit != 0 {
			out = append(out, e)
		}
	}
	return out
}

// DecodeEvents returns the named interrupt sources present in an ISR value.
func DecodeEvents(isr uint32) []Event {
	return filterEvents([]Event{
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
	}, isr)
}

// DecodeCtrl returns the named control bits set in a CTRL register value.
func DecodeCtrl(ctrl uint32) []Event {
	return filterEvents([]Event{
		{ctrlUpstreamConnect, "UPSTREAM_CONNECT"},
		{ctrlFullSpeedOnly, "FULL_SPEED_ONLY"},
		{ctrlClkStopSuspend, "CLK_STOP_SUSPEND"},
		{ctrlAutoRemoteWakeup, "AUTO_REMOTE_WAKEUP"},
		{ctrlPHYResetDis, "PHY_RESET_DIS"},
		{ctrlSplitIn, "SPLIT_IN"},
		{ctrlISORspCtrl, "ISO_RSP_CTRL"},
		{ctrlLongDesc, "LONG_DESC"},
		{ctrlEnlargeFIFO, "ENLARGE_FIFO"},
		{ctrlPHYClk, "PHY_CLK"},
	}, ctrl)
}

// DecodePHY returns the named PHY_CTRL bits set in a value.
func DecodePHY(phy uint32) []Event {
	return filterEvents([]Event{
		{phyCPUDieSRAMEn, "CPU_DIE_SRAM_EN"},
		{phyIODieAHBAddr34, "IO_DIE_AHBM_ADDR34"},
		{phyIODieSRAMEn, "IO_DIE_SRAM_EN"},
		{phyFIFOForceRetry, "FIFO_FORCE_RETRY"},
	}, phy)
}

// MuxMode classifies a port function-mux register value for the given port:
// "device" (gadget, the value we program), "host/other", or "n/a" if the port
// has no mux. The raw masked value is returned for display.
func (p Port) MuxMode(funcMuxVal uint32) (mode string, masked uint32) {
	if p.FuncMux == 0 {
		return "n/a", 0
	}
	masked = funcMuxVal & p.FuncMask
	if masked == p.FuncBits {
		return "device", masked
	}
	return "host/other", masked
}
