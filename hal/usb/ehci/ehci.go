// Package ehci provides an ASPEED AST2700 EHCI USB-2.0 host controller
// bring-up driver.
//
// Unlike the vHub gadget controller (hal/usb/vhub), this is the *host* side:
// it drives a standard EHCI operational register set behind the ASPEED SCU
// clock/reset/routing wrapper so an external USB device (mass storage, HID)
// plugged into the board's physical port can be detected, reset and — once the
// transfer layers land — enumerated and read.
//
// The predefined target is die1 Port D (`ehci3` @ 0x1412_3000), which on the
// AST2700 DC-SCM is wired to the card's physical USB Type-A receptacle (the
// USB2 D+/D- pair of the cross-port USB3 connector). A SuperSpeed stick in that
// receptacle falls back to high speed on this path. Port D lives on die1, so
// its clock gate is in SCU1 CLK_STOP2 (0x260) and its routing in SCU1
// USB_MULTI_CTRL (0x3B0) — both distinct from the die0 vHub wiring.
//
// This is a bring-up / diagnostic driver: it takes the controller out of reset,
// routes the port to the host, tunes the shared USB2 PHY, resets the host
// controller, powers the port and reports PORTSC. EP0/transfer rings, DMA and
// device enumeration are not yet implemented (that is the next slice).
//
// Register offsets follow the EHCI 1.0 specification; the SCU wrapper follows
// the Linux ast2700 clk/reset drivers and the vendor u-boot ast_usb wiring.
//
// Intended for GOOS=tamago GOARCH=arm64 on the CA35.
package ehci

import (
	"time"

	"github.com/kyanitecomputer/aspeed-go/reg"
)

// EHCI capability registers, relative to the controller base.
const (
	capCAPLENGTH  = 0x00 // byte 0: operational-register offset; [31:16] HCIVERSION.
	capHCSPARAMS  = 0x04 // structural parameters (N_PORTS, port power control).
	capHCCPARAMS  = 0x08 // capability parameters (64-bit addressing, etc.).
)

// HCSPARAMS (0x04) fields.
const (
	hcsNPortsMask = 0xf      // [3:0] number of downstream ports.
	hcsPPC        = 1 << 4   // port power control: ports need explicit power-on.
)

// EHCI operational registers, relative to (base + CAPLENGTH).
const (
	opUSBCMD          = 0x00
	opUSBSTS          = 0x04
	opUSBINTR         = 0x08
	opFRINDEX         = 0x0C
	opCTRLDSSEGMENT   = 0x10
	opPERIODICLIST    = 0x14
	opASYNCLIST       = 0x18
	opCONFIGFLAG      = 0x40
	opPORTSC0         = 0x44 // first port; further ports at +4 each.
)

// USBCMD (op 0x00) bits.
const (
	cmdRunStop     = 1 << 0
	cmdHCReset     = 1 << 1
	cmdPeriodicEn  = 1 << 4
	cmdAsyncEn     = 1 << 5
)

// USBSTS (op 0x04) bits.
const (
	stsPortChange = 1 << 2
	stsHCHalted   = 1 << 12
)

// CONFIGFLAG (op 0x40) bits.
const cfgConfigure = 1 << 0 // route all ports to this EHCI controller.

// PORTSC (op 0x44) bits.
const (
	portConnect       = 1 << 0  // current connect status.
	portConnectChange = 1 << 1  // connect status change (write 1 to clear).
	portEnable        = 1 << 2  // port enabled (set by HC after a HS reset).
	portEnableChange  = 1 << 3  // write 1 to clear.
	portOverCurrent   = 1 << 4
	portOverCurChange = 1 << 5  // write 1 to clear.
	portResume        = 1 << 6  // force port resume.
	portSuspend       = 1 << 7
	portReset         = 1 << 8  // port reset drive.
	portLineStatusSh  = 10      // [11:10] D+/D- line state.
	portLineStatusMsk = 0x3 << 10
	portPower         = 1 << 12 // port power.
	portOwner         = 1 << 13 // 1 = owned by companion (UHCI), 0 = EHCI.

	// Write-1-to-clear change bits; preserved-through read-modify-write masks
	// must exclude these so an RMW does not accidentally clear a change.
	portRWC = portConnectChange | portEnableChange | portOverCurChange
)

// lineStatus classifies the PORTSC D+/D- line state. K-state on a connected,
// not-yet-enabled port means a low-speed device that EHCI must hand to the
// companion controller.
const (
	lineSE0     = 0x0 // both low.
	lineKState  = 0x1 // low-speed device.
	lineJState  = 0x2 // full/high-speed device.
	lineUndef   = 0x3
)

// USB2 PHY control window, relative to Port.PHYBase. The per-port USB2 PHY
// lives inside the sibling vHub register block (for Port D that is vhubd, PHY
// window @ 0x1412_2800). It shares the PORTD_VHUB_EHCI reset with the EHCI
// controller. PHY_CTL_STS_1 (+0x00) carries the SRAM-access enable; STS_2/3
// carry the analog clock-rate and pre-emphasis tuning.
//
// Crucially, the PHY does not clock from reset-release alone: the owning vHub's
// CTRL register (PHY_CLK | PHY_RESET_DIS) and its SRAM-access enable must be set
// to bring the PHY up, exactly as for the die0 vHub gadget path. No Linux driver
// does this for the generic-ehci host, so the bring-up must do it here.
const (
	phyCtlSts1 = 0x00 // SRAM-access enable + AHB master address bits.
	phyCtlSts2 = 0x04 // reference clock rate select.
	phyCtlSts3 = 0x08 // pre-emphasis current.

	phyCtlSts2ClkRateMask = 0x3 << 26
	phyCtlSts2ClkRate60M  = 0x3 << 26 // b'11 = 60 MHz.
	phyCtlSts3PreEmphMask = 0x3 << 21
	phyCtlSts3PreEmph2    = 0x2 << 21 // b'10 = setting 2.

	// PHY_CTL_STS_1 (0x800) SRAM-access enable bits. Port C/D are on the IO die.
	phyIODieAHBAddr34 = 1 << 5  // IO-die AHB master address bit 34.
	phyIODieSRAMEn    = 1 << 10 // IO-die SRAM access enable.
)

// Sibling vHub CTRL register (base + 0x00) bits used to power the shared PHY.
const (
	vhubCtrl         = 0x00
	vhubCtrlPHYReset = 1 << 11 // PHY_RESET_DIS: hold the internal PHY reset off.
	vhubCtrlPHYClk   = 1 << 31 // PHY_CLK: enable the PHY clock.
)

// SCU register offsets. Resets for USB live in RST_CTRL2 (0x220 assert /
// 0x224 clear) on both dies; the clock-stop register differs per die (die0
// CLK_STOP @ 0x240, die1 CLK_STOP2 @ 0x260) and is carried in Port.ClkStopReg.
// Both use a paired clear register at +0x04 to enable (clear the stop bit).
const (
	scuProtect     = 0x000
	scuRstCtrl2    = 0x220
	scuRstCtrl2Clr = 0x224

	scuProtectKey = 0x1688a8a8
)

// Port describes one EHCI host controller instance and its SCU wiring.
type Port struct {
	Name    string
	Base    uint32 // EHCI register base.
	SCUBase uint32 // controlling SCU (SCU1 @ 0x14c0_2000 for die1).
	IRQ     int    // interrupt controller line (reference; polled for now).

	ClkStopReg uint32 // SCU clock-stop register offset (0x240 die0, 0x260 die1).
	ClockBit   uint32 // clock-gate bit; enabled by writing to ClkStopReg+0x04.
	ResetBit   uint32 // RST_CTRL2 (0x220) bit; assert at 0x220, release at 0x224.

	FuncMux  uint32 // SCU USB routing register offset (0 = skip).
	FuncMask uint32 // routing field mask.
	FuncBits uint32 // routing value selecting host (EHCI) mode.

	// PHYBase is the shared USB2 PHY control window (PHY_CTL_STS_*), gated by the
	// same ResetBit; 0 to skip PHY tuning.
	PHYBase uint32

	// VHubBase is the sibling vHub controller that owns the shared USB2 PHY
	// (vhubd for Port D). Its CTRL register must be written to clock the PHY and
	// hold its reset off; 0 to skip PHY power-up. IODie selects the IO-die
	// SRAM-enable bits in PHY_CTL_STS_1.
	VHubBase uint32
	IODie    bool
}

// EHCI3PortD is the DC-SCM physical USB port on die1 Port D: the EHCI host
// controller `ehci3`, routed to the USB2 host pins (USB2DH), sharing the Port D
// USB2 PHY inside vhubd. This is the controller a stick in the card's Type-A
// receptacle appears on (at high speed).
var EHCI3PortD = Port{
	Name: "ehci3", Base: 0x14123000, SCUBase: 0x14c02000, IRQ: 29,
	ClkStopReg: 0x260, ClockBit: 1 << 18, // SCU1 CLK_STOP2 PORTDUSB2CLK.
	ResetBit: 1 << 29, // SCU1 RST_CTRL2 PORTD_VHUB_EHCI.
	FuncMux:  0x3b0, FuncMask: 0x3 << 2, FuncBits: 0x2 << 2, // USB2DH (host).
	PHYBase:  0x14122800, // Port D USB2 PHY window inside vhubd.
	VHubBase: 0x14122000, // vhubd controller (owns/clocks the Port D PHY).
	IODie:    true,       // die1 IO-die SRAM-enable bits.
}

// Controller is a single AST2700 EHCI host controller.
type Controller struct {
	port   Port
	caplen uint32 // operational-register offset (cached after Init).
	dma    DMA    // transfer-layer DMA backend (nil until SetDMA).
}

// New returns a controller for the given port.
func New(p Port) *Controller { return &Controller{port: p} }

// Port returns the controller's port descriptor.
func (c *Controller) Port() Port { return c.port }

func (c *Controller) r(off uint32) uint32    { return reg.Read(c.port.Base + off) }
func (c *Controller) w(off uint32, v uint32) { reg.Write(c.port.Base + off, v) }

// op reads an operational register (relative to base+CAPLENGTH).
func (c *Controller) op(off uint32) uint32 { return reg.Read(c.port.Base + c.caplen + off) }

// opw writes an operational register.
func (c *Controller) opw(off, v uint32) { reg.Write(c.port.Base+c.caplen+off, v) }

func (c *Controller) scuUnlock() { reg.Write(c.port.SCUBase+scuProtect, scuProtectKey) }
func (c *Controller) scuLock()   { reg.Write(c.port.SCUBase+scuProtect, 1) }

// Step records one observable bring-up action for diagnostics: the register it
// touched, its value before and after the write, and the bits expected to
// change. Mirrors vhub.Step so the console can render both the same way.
type Step struct {
	Name    string
	RegName string
	Addr    uint32
	Before  uint32
	After   uint32
	WantSet uint32
	WantClr uint32
	Note    string
}

// Checked reports whether the step has an expected outcome to verify.
func (s Step) Checked() bool { return s.WantSet != 0 || s.WantClr != 0 }

// OK reports whether the observed After value matches the expectation.
func (s Step) OK() bool {
	if s.WantSet != 0 && s.After&s.WantSet != s.WantSet {
		return false
	}
	if s.WantClr != 0 && s.After&s.WantClr != 0 {
		return false
	}
	return true
}

// InitSteps runs the full host-controller bring-up — SCU host-function routing,
// reset assert, clock enable, PLL wait, reset deassert (controller + shared
// USB2 PHY), PHY tuning, HC reset, port routing (CONFIGFLAG) and port power —
// returning one Step per action with before/after register values and
// expectations. After this the controller is halted with the port powered;
// call ResetPort to drive a device reset and enable the port. This is the
// single source of truth for bring-up; Init runs it and discards the trace.
func (c *Controller) InitSteps() []Step {
	p := c.port
	steps := make([]Step, 0, 12)
	rd := func(off uint32) uint32 { return reg.Read(p.SCUBase + off) }

	// Keep the SCU unlocked for the whole clock/reset/route sequence.
	lockBefore := rd(scuProtect)
	c.scuUnlock()
	steps = append(steps, Step{
		Name: "unlock SCU", RegName: "SCU_PROTECT", Addr: p.SCUBase + scuProtect,
		Before: lockBefore, After: rd(scuProtect),
		Note: "write key 0x1688a8a8 (0x000 reads back the silicon revision id)",
	})

	// Route the port to the host (EHCI) function.
	if p.FuncMux != 0 {
		before := rd(p.FuncMux)
		reg.MaskWrite32(uintptr(p.SCUBase+p.FuncMux), p.FuncMask, p.FuncBits)
		steps = append(steps, Step{
			Name: "route port to host (EHCI) function", RegName: "SCU_USB_MULTI_CTRL",
			Addr: p.SCUBase + p.FuncMux, Before: before, After: rd(p.FuncMux),
			WantSet: p.FuncBits, WantClr: p.FuncMask &^ p.FuncBits,
			Note: "port D field [3:2]=2 (USB2DH): PHY routed to the EHCI host",
		})
	}

	// Assert reset.
	{
		before := rd(scuRstCtrl2)
		reg.Write(p.SCUBase+scuRstCtrl2, p.ResetBit)
		steps = append(steps, Step{
			Name: "assert controller reset", RegName: "SCU_RST_CTRL2",
			Addr: p.SCUBase + scuRstCtrl2, Before: before, After: rd(scuRstCtrl2),
			WantSet: p.ResetBit, Note: "held while the clock spins up",
		})
	}

	// Enable the port clock (clear its stop bit) in the die-specific CLK_STOP.
	{
		before := rd(p.ClkStopReg)
		reg.Write(p.SCUBase+p.ClkStopReg+0x04, p.ClockBit)
		steps = append(steps, Step{
			Name: "enable port clock (clear stop)", RegName: "SCU_CLK_STOP",
			Addr: p.SCUBase + p.ClkStopReg, Before: before, After: rd(p.ClkStopReg),
			WantClr: p.ClockBit, Note: "stop bit must read 0 = clock running (die1 CLK_STOP2)",
		})
	}

	time.Sleep(10 * time.Millisecond) // PLL lock
	steps = append(steps, Step{Name: "wait 10ms for PLL lock"})

	// Deassert reset (controller and its shared USB2 PHY, both PORTD_VHUB_EHCI).
	{
		before := rd(scuRstCtrl2)
		reg.Write(p.SCUBase+scuRstCtrl2Clr, p.ResetBit)
		steps = append(steps, Step{
			Name: "deassert controller + PHY reset", RegName: "SCU_RST_CTRL2",
			Addr: p.SCUBase + scuRstCtrl2, Before: before, After: rd(scuRstCtrl2),
			WantClr: p.ResetBit, Note: "releases the EHCI core and the Port D USB2 PHY",
		})
	}
	c.scuLock()
	time.Sleep(time.Millisecond)

	// Tune the shared USB2 PHY (inside vhubd): reference clock rate and
	// pre-emphasis, matching the vendor usb_usb2_init.
	if p.PHYBase != 0 {
		{
			addr := p.PHYBase + phyCtlSts2
			before := reg.Read(addr)
			reg.MaskWrite32(uintptr(addr), phyCtlSts2ClkRateMask, phyCtlSts2ClkRate60M)
			steps = append(steps, Step{
				Name: "set USB2 PHY clock rate (60MHz)", RegName: "PHY_CTL_STS_2",
				Addr: addr, Before: before, After: reg.Read(addr),
				WantSet: phyCtlSts2ClkRate60M, Note: "PHY [27:26]=b11 (inside vhubd)",
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
				Note: "PHY [22:21]=b10 (inside vhubd)",
			})
		}
	}

	// Power up the shared USB2 PHY through the sibling vHub that owns it. The PHY
	// tuning above only sets analog trims; the PHY does not actually clock until
	// its owning vHub's SRAM-access is enabled and its CTRL PHY_CLK/PHY_RESET_DIS
	// bits are set. Without this the EHCI port never sees a device connect.
	if p.VHubBase != 0 {
		{
			addr := p.VHubBase + 0x800 + phyCtlSts1
			want := uint32(phyIODieSRAMEn | phyIODieAHBAddr34)
			if !p.IODie {
				want = 1 << 4 // CPU-die SRAM enable.
			}
			before := reg.Read(addr)
			reg.Write(addr, before|want)
			steps = append(steps, Step{
				Name: "enable PHY SRAM access (sibling vHub)", RegName: "PHY_CTL_STS_1",
				Addr: addr, Before: before, After: reg.Read(addr),
				WantSet: want, Note: "IO-die SRAM access bits in vhubd's PHY window",
			})
		}
		{
			addr := p.VHubBase + vhubCtrl
			before := reg.Read(addr)
			reg.Write(addr, before|vhubCtrlPHYClk|vhubCtrlPHYReset)
			steps = append(steps, Step{
				Name: "clock PHY on (sibling vHub CTRL)", RegName: "VHUB_CTRL",
				Addr: addr, Before: before, After: reg.Read(addr),
				WantSet: vhubCtrlPHYClk | vhubCtrlPHYReset,
				Note: "PHY_CLK + PHY_RESET_DIS in vhubd bring the shared USB2 PHY up",
			})
		}
		time.Sleep(5 * time.Millisecond) // let the PHY clock settle
	}

	// Cache the operational-register offset (CAPLENGTH). Reads garbage if the
	// controller is unclocked; the HC reset below is the real reachability test.
	capword := c.r(capCAPLENGTH)
	c.caplen = capword & 0xff
	steps = append(steps, Step{
		Name: "read EHCI capability registers", RegName: "CAPLENGTH/HCIVERSION",
		Addr: p.Base + capCAPLENGTH, Before: capword, After: capword,
		Note: "CAPLENGTH = op-register offset; HCIVERSION in [31:16]",
	})

	// Host controller reset: set HCRESET and wait for the HC to self-clear it.
	// A completing HCRESET is the reliable proof the core is clocked and mapped.
	{
		before := c.op(opUSBCMD)
		c.opw(opUSBCMD, before|cmdHCReset)
		var after uint32
		for i := 0; i < 100; i++ {
			after = c.op(opUSBCMD)
			if after&cmdHCReset == 0 {
				break
			}
			time.Sleep(time.Millisecond)
		}
		steps = append(steps, Step{
			Name: "reset host controller (HCRESET)", RegName: "USBCMD",
			Addr: p.Base + c.caplen + opUSBCMD, Before: before, After: after,
			WantClr: cmdHCReset,
			Note: "HC self-clears HCRESET when done; if it stays set the core is unclocked",
		})
	}

	// After reset the controller must be halted.
	{
		sts := c.op(opUSBSTS)
		steps = append(steps, Step{
			Name: "check controller halted after reset", RegName: "USBSTS",
			Addr: p.Base + c.caplen + opUSBSTS, Before: sts, After: sts,
			WantSet: stsHCHalted, Note: "HCHalted=1 expected post-reset",
		})
	}

	// Route all ports to this EHCI controller (CONFIGFLAG=1).
	{
		before := c.op(opCONFIGFLAG)
		c.opw(opCONFIGFLAG, cfgConfigure)
		steps = append(steps, Step{
			Name: "route ports to EHCI (CONFIGFLAG)", RegName: "CONFIGFLAG",
			Addr: p.Base + c.caplen + opCONFIGFLAG, Before: before, After: c.op(opCONFIGFLAG),
			WantSet: cfgConfigure, Note: "hand ports to EHCI rather than the companion",
		})
	}

	// Power the port if the controller has port-power control.
	if c.op(0) != 0xffffffff && reg.Read(p.Base+capHCSPARAMS)&hcsPPC != 0 {
		before := c.op(opPORTSC0)
		c.opw(opPORTSC0, (before&^portRWC)|portPower)
		steps = append(steps, Step{
			Name: "power port", RegName: "PORTSC",
			Addr: p.Base + c.caplen + opPORTSC0, Before: before, After: c.op(opPORTSC0),
			WantSet: portPower, Note: "HCSPARAMS.PPC set: port needs explicit power-on",
		})
		time.Sleep(20 * time.Millisecond) // let power settle before sampling connect
	}

	// Start the controller running so it samples port status.
	{
		before := c.op(opUSBCMD)
		c.opw(opUSBCMD, before|cmdRunStop)
		steps = append(steps, Step{
			Name: "start controller (RUN)", RegName: "USBCMD",
			Addr: p.Base + c.caplen + opUSBCMD, Before: before, After: c.op(opUSBCMD),
			WantSet: cmdRunStop, Note: "HC begins sampling PORTSC connect status",
		})
	}
	time.Sleep(5 * time.Millisecond)

	return steps
}

// Init performs the full host-controller bring-up (SCU route/clock/reset, PHY
// tune, HC reset, port power, run). Use InitSteps for a diagnostic trace.
func (c *Controller) Init() { c.InitSteps() }

// PollConnect polls the root port for a device connection for up to d,
// returning the final PORTSC, whether a device connected, and how long it took.
// A high-speed-capable USB device asserts its D+ pull-up almost immediately,
// but a SuperSpeed device plugged into a cross-port receptacle first attempts
// (and times out) SuperSpeed link training before falling back to USB2 and
// asserting the pull-up — which can take a second or more — so a single sample
// right after RUN is unreliable.
func (c *Controller) PollConnect(d time.Duration) (portsc uint32, connected bool, waited time.Duration) {
	start := time.Now()
	deadline := start.Add(d)
	for {
		portsc = c.op(opPORTSC0)
		if portsc&portConnect != 0 {
			return portsc, true, time.Since(start)
		}
		if !time.Now().Before(deadline) {
			return portsc, false, time.Since(start)
		}
		time.Sleep(20 * time.Millisecond)
	}
}

// clearChange acknowledges the given PORTSC write-1-to-clear change bits without
// perturbing the read/write control bits (power, enable) or triggering a reset.
func (c *Controller) clearChange(bits uint32) {
	cur := c.op(opPORTSC0)
	c.opw(opPORTSC0, (cur&^portRWC&^portReset)|(bits&portRWC))
}

// ResetAttempt records the key PORTSC read-backs around one reset pulse so a
// caller can tell a write that never landed (PR does not read back set) from a
// high-speed handshake that failed (PR asserts and clears but the port never
// enables).
type ResetAttempt struct {
	Written     uint32 // value written to assert PortReset.
	PRAsserted  uint32 // PORTSC read back immediately after writing PR=1.
	DuringReset uint32 // PORTSC ~50ms in, PR still held.
	AfterClear  uint32 // PORTSC right after writing PR=0.
	Settled     uint32 // PORTSC after the post-reset settle poll.
}

// ResetResult captures one or more reset attempts and the final verdict.
type ResetResult struct {
	Before   uint32
	Attempts []ResetAttempt
	After    uint32
	Enabled  bool
	Speed    string
}

// resetOnce drives one USB reset pulse, capturing PORTSC at each key point:
// right after asserting PR (to confirm the write landed), mid-reset, right
// after releasing PR, and after a settle poll that waits for PR to clear and
// the port to enable.
func (c *Controller) resetOnce() ResetAttempt {
	var a ResetAttempt
	ps := c.op(opPORTSC0)
	a.Written = (ps &^ portRWC &^ portEnable) | portReset

	c.opw(opPORTSC0, a.Written)
	a.PRAsserted = c.op(opPORTSC0)
	time.Sleep(50 * time.Millisecond)
	a.DuringReset = c.op(opPORTSC0)

	// Clear PortReset from a FRESH read, preserving every other control bit and
	// not touching the write-1-to-clear change bits. This is critical: the
	// ASPEED EHCI enables the port during the reset (PED reads 1 mid-reset), and
	// in EHCI writing 0 to Port Enable *disables* the port — so clearing PR from
	// the stale pre-reset value (PED=0) would immediately undo the enable. Mirror
	// Linux ehci: temp &= ~(PORT_RWC_BITS | PORT_RESET).
	cur := c.op(opPORTSC0)
	c.opw(opPORTSC0, cur&^portRWC&^portReset)
	a.AfterClear = c.op(opPORTSC0)

	// Settle: wait for the HC to clear PR (<=2ms spec) and the port to enable.
	for i := 0; i < 30; i++ {
		time.Sleep(10 * time.Millisecond)
		s := c.op(opPORTSC0)
		if s&portReset == 0 && (s&portEnable != 0 || s&portConnect == 0) {
			a.Settled = s
			return a
		}
	}
	a.Settled = c.op(opPORTSC0)
	return a
}

// ResetPort resets the root port and reports how it settled. It debounces the
// connection (USB 2.0 §7.1.7.3), clears the connect-change, then drives up to
// three reset pulses, stopping as soon as the port enables. Only meaningful
// once a device is connected (PORTSC.CCS set).
func (c *Controller) ResetPort() ResetResult {
	var r ResetResult
	r.Before = c.op(opPORTSC0)

	time.Sleep(100 * time.Millisecond) // connect debounce
	c.clearChange(portConnectChange)

	for i := 0; i < 3; i++ {
		a := c.resetOnce()
		r.Attempts = append(r.Attempts, a)
		if a.Settled&portEnable != 0 || a.Settled&portConnect == 0 {
			break
		}
	}

	r.After = c.op(opPORTSC0)
	r.Enabled = r.After&portEnable != 0
	r.Speed = c.Status().Speed()
	return r
}

// Status is a snapshot of the controller and its SCU wiring for diagnostics.
type Status struct {
	CapLength  uint32
	HCIVersion uint32
	HCSParams  uint32
	HCCParams  uint32

	USBCmd  uint32
	USBSts  uint32
	ConfigF uint32
	PortSC  uint32

	SCUClkStop uint32
	SCUReset   uint32
	SCUFuncMux uint32

	HasPHY     bool
	PHYCtlSts1 uint32
	PHYCtlSts2 uint32
	PHYCtlSts3 uint32

	HasVHub  bool
	VHubCtrl uint32 // sibling vHub CTRL (PHY_CLK/PHY_RESET_DIS state).
}

// Status reads the controller and SCU registers into a snapshot. It uses the
// cached CAPLENGTH from a prior Init/InitSteps; if none has run it reads it
// fresh so a read-only `status` still resolves the operational registers.
func (c *Controller) Status() Status {
	p := c.port
	if c.caplen == 0 {
		if cl := c.r(capCAPLENGTH) & 0xff; cl != 0 && cl != 0xff {
			c.caplen = cl
		}
	}
	s := Status{
		CapLength:  c.r(capCAPLENGTH) & 0xff,
		HCIVersion: c.r(capCAPLENGTH) >> 16,
		HCSParams:  c.r(capHCSPARAMS),
		HCCParams:  c.r(capHCCPARAMS),
		USBCmd:     c.op(opUSBCMD),
		USBSts:     c.op(opUSBSTS),
		ConfigF:    c.op(opCONFIGFLAG),
		PortSC:     c.op(opPORTSC0),
		SCUClkStop: reg.Read(p.SCUBase + p.ClkStopReg),
		SCUReset:   reg.Read(p.SCUBase + scuRstCtrl2),
	}
	if p.FuncMux != 0 {
		s.SCUFuncMux = reg.Read(p.SCUBase + p.FuncMux)
	}
	if p.PHYBase != 0 {
		s.HasPHY = true
		s.PHYCtlSts1 = reg.Read(p.PHYBase + phyCtlSts1)
		s.PHYCtlSts2 = reg.Read(p.PHYBase + phyCtlSts2)
		s.PHYCtlSts3 = reg.Read(p.PHYBase + phyCtlSts3)
	}
	if p.VHubBase != 0 {
		s.HasVHub = true
		s.VHubCtrl = reg.Read(p.VHubBase + vhubCtrl)
	}
	return s
}

// PHYClocked reports whether the shared USB2 PHY has been clocked via the
// sibling vHub CTRL (PHY_CLK + PHY_RESET_DIS both set).
func (s Status) PHYClocked() bool {
	return s.VHubCtrl&(vhubCtrlPHYClk|vhubCtrlPHYReset) == (vhubCtrlPHYClk | vhubCtrlPHYReset)
}

// NPorts returns the number of root ports the controller reports.
func (s Status) NPorts() uint32 { return s.HCSParams & hcsNPortsMask }

// PortPowerControl reports whether ports require explicit power-on.
func (s Status) PortPowerControl() bool { return s.HCSParams&hcsPPC != 0 }

// Running reports whether the controller is running (not stopped).
func (s Status) Running() bool { return s.USBCmd&cmdRunStop != 0 }

// Halted reports whether the controller is halted.
func (s Status) Halted() bool { return s.USBSts&stsHCHalted != 0 }

// Configured reports whether the ports are routed to EHCI (CONFIGFLAG set).
func (s Status) Configured() bool { return s.ConfigF&cfgConfigure != 0 }

// DeviceConnected reports whether a device is present on the root port.
func (s Status) DeviceConnected() bool { return s.PortSC&portConnect != 0 }

// PortEnabled reports whether the root port is enabled (a high-speed device
// completed reset).
func (s Status) PortEnabled() bool { return s.PortSC&portEnable != 0 }

// PortPowered reports whether the root port is powered.
func (s Status) PortPowered() bool { return s.PortSC&portPower != 0 }

// OwnedByCompanion reports whether the port has been released to the companion
// (UHCI) controller — the case for a low/full-speed device.
func (s Status) OwnedByCompanion() bool { return s.PortSC&portOwner != 0 }

// Speed classifies the attached device from PORTSC. A device that has completed
// a high-speed reset shows Port Enabled; before that the D+/D- line state
// distinguishes a low-speed (K) from a full/high-speed-capable (J) device.
func (s Status) Speed() string {
	if s.PortSC&portConnect == 0 {
		return "no device"
	}
	if s.PortSC&portEnable != 0 {
		return "high-speed"
	}
	switch (s.PortSC & portLineStatusMsk) >> portLineStatusSh {
	case lineKState:
		return "low-speed (needs companion)"
	case lineJState:
		return "full-speed (needs companion) or pre-reset high-speed"
	case lineSE0:
		return "SE0 (idle/reset)"
	default:
		return "undefined line state"
	}
}

// ClockRunning reports whether the port clock gate is enabled (stop bit clear).
func (s Status) ClockRunning(p Port) bool { return s.SCUClkStop&p.ClockBit == 0 }

// InReset reports whether the controller reset is currently asserted.
func (s Status) InReset(p Port) bool { return s.SCUReset&p.ResetBit != 0 }

// Event describes one decodable status bit for diagnostics.
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

// DecodeCmd returns the named USBCMD bits set in a value.
func DecodeCmd(cmd uint32) []Event {
	return filterEvents([]Event{
		{cmdRunStop, "RUN"},
		{cmdHCReset, "HCRESET"},
		{cmdPeriodicEn, "PERIODIC_EN"},
		{cmdAsyncEn, "ASYNC_EN"},
	}, cmd)
}

// DecodeSts returns the named USBSTS bits set in a value.
func DecodeSts(sts uint32) []Event {
	return filterEvents([]Event{
		{stsPortChange, "PORT_CHANGE"},
		{stsHCHalted, "HC_HALTED"},
	}, sts)
}

// DecodePortSC returns the named PORTSC bits set in a value.
func DecodePortSC(portsc uint32) []Event {
	return filterEvents([]Event{
		{portConnect, "CONNECT"},
		{portConnectChange, "CONNECT_CHANGE"},
		{portEnable, "ENABLED"},
		{portEnableChange, "ENABLE_CHANGE"},
		{portOverCurrent, "OVERCURRENT"},
		{portSuspend, "SUSPEND"},
		{portReset, "RESET"},
		{portPower, "POWER"},
		{portOwner, "COMPANION_OWNED"},
	}, portsc)
}

// MuxMode classifies a routing-register value for the port: "host" (the value
// we program), "device/other", or "n/a" if the port has no routing field.
func (p Port) MuxMode(funcMuxVal uint32) (mode string, masked uint32) {
	if p.FuncMux == 0 {
		return "n/a", 0
	}
	masked = funcMuxVal & p.FuncMask
	if masked == p.FuncBits {
		return "host", masked
	}
	return "device/other", masked
}
