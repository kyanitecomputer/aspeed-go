// Package aspeedgfx provides an ASPEED SOC display controller driver.
package aspeedgfx

import (
	"fmt"
	"time"

	"src.kyanite.computer/aspeed-go/hal/framebuffer"
	"src.kyanite.computer/aspeed-go/reg"
)

const (
	AST2700GFXBase   = 0x12c09000
	AST2700SCU0Base  = 0x12c02000
	AST2700SCU1Base  = 0x14c02000
	AST2700DPBase    = 0x12c0a000
	AST2700DPMCUBase = 0x11000000
	AST2700GFXIRQ    = 32 + 14

	offSCUProtect   = 0x000
	offSCUReset1    = 0x200
	offSCUReset2    = 0x220
	offSCUClockStop = 0x240
	offSCUClockClr  = 0x244
	offSCUClockSel2 = 0x288
	offSCUDAC       = 0x414
	offSCUPCIE0DP   = 0x900
	offSCUPCIE1DP   = 0x910

	offSCU1DPDAC = 0x0d0
	offSCU1DPCLK = 0x320

	offMCUCtrl  = 0x100e0
	offMCUInt   = 0x100e8
	offMCUDE0   = 0x00de0
	offMCUReDrv = 0x00e00

	mcuCtrlAHBSIMEMEn  = 1 << 0
	mcuCtrlAHBSSWRst   = 1 << 4
	mcuCtrlAHBMSWRst   = 1 << 8
	mcuCtrlCoreSWRst   = 1 << 12
	mcuCtrlDMEMShut    = 1 << 16
	mcuCtrlDMEMClkOff  = 1 << 18
	mcuCtrlIMEMShut    = 1 << 20
	mcuCtrlIMEMClkOff  = 1 << 22
	mcuCtrlConfig      = 1 << 28
	mcuInterruptEnable = 0xff << 16

	packerCPU  = 0x12c1d000
	retimerCPU = 0x12c1d100
	packerIO   = 0x14c3a000
	retimerIO  = 0x14c3a100

	offCtrl1   = 0x60
	offCtrl2   = 0x64
	offStatus  = 0x68
	offHoriz0  = 0x70
	offHoriz1  = 0x74
	offVert0   = 0x78
	offVert1   = 0x7c
	offAddr    = 0x80
	offOffset  = 0x84
	offThrod   = 0x88
	offCursor0 = 0x90
	offCursor1 = 0x94

	ctrlEnable    = 1 << 0
	ctrlCursor    = 1 << 1
	ctrlOSD       = 1 << 2
	ctrlColorMask = 0x7 << 7
	ctrlXRGB8888  = 0x2 << 7
	ctrlRGB565    = 0x0 << 7

	CTRL_XRGB8888 = ctrlXRGB8888
	ctrlVBlankEn  = 1 << 30
	ctrlVBlankSts = 1 << 31

	ctrlDACEnable    = 1 << 0
	dpControlFromSOC = 1<<24 | 1<<28
	// dpReadyBit is set by the DPMCU firmware in the DP scratch register when
	// link training completes ("executing"). The low status byte varies with
	// the host handshake bits (e.g. 0x2e or 0x3e); bit 13 is the reliable
	// readiness signal (matches the vendor bring-up and the RoT wait_dp_ready).
	dpReadyBit      = 1 << 13
	dpLocatedPCIE1  = 1 << 8
	dpResolution800 = 0x01050020

	defaultThreshold = 0x70<<8 | 0x50
	scanLineMax      = 128
	scuProtectKey    = 0x1688a8a8
)

// Mode describes a display mode.
//
// Sync positions are absolute pixel/line coordinates: HSyncStart/HSyncEnd are
// measured from the start of the active line, VSyncStart/VSyncEnd from the
// start of the active frame. PixelClockKHz is the dot clock and ModeIndex is
// the ASTDP firmware video-format index consumed by the DPMCU (see
// DisplayFormat).
type Mode struct {
	Width      int
	Height     int
	HTotal     int
	HSyncStart int
	HSyncEnd   int
	VTotal     int
	VSyncStart int
	VSyncEnd   int

	PixelClockKHz int
	ModeIndex     uint8
}

// Mode800x600 is the AST2700 Linux driver default DP mode.
var Mode800x600 = Mode{Width: 800, Height: 600, HTotal: 1056, HSyncStart: 840, HSyncEnd: 968, VTotal: 628, VSyncStart: 601, VSyncEnd: 605, PixelClockKHz: 40000, ModeIndex: ASTDP_800x600_60}

// Controller is an ASPEED SOC display controller.
type Controller struct {
	Base uint32
}

// Status contains diagnostic display-controller and DP state.
type Status struct {
	Ctrl1      uint32
	Ctrl2      uint32
	CRTCStatus uint32
	Addr       uint32
	Offset     uint32
	Horiz0     uint32
	Horiz1     uint32
	Vert0      uint32
	Vert1      uint32
	DPSource   uint32
	DPMCU      uint32
	DPMCUCtrl  uint32
	DPMCUInt   uint32
	DPVersion  uint32
	ReDriver   uint32
	SCUDAC     uint32
	PCIE0DP    uint32
	PCIE1DP    uint32
	DPReady    bool
}

// NewFramebuffer creates a framebuffer view and programs the controller.
func (c *Controller) NewFramebuffer(mode Mode, format framebuffer.PixelFormat, addr uintptr) (*framebuffer.Framebuffer, error) {
	if mode.Width <= 0 || mode.Height <= 0 {
		return nil, fmt.Errorf("aspeedgfx: invalid mode")
	}
	fb := framebuffer.New(framebuffer.Config{Width: mode.Width, Height: mode.Height, Format: format, PhysAddr: addr})
	c.SetMode(mode, fb)
	return fb, nil
}

// SanitizeCRT zeros CRT control registers to clear stale BootMCU/VBIOS state.
func (c *Controller) SanitizeCRT() {
	reg.Write(c.Base+offCtrl1, 0)
	reg.Write(c.Base+offCtrl2, 0)
}

// SetMode programs scanout timing and framebuffer address.
func (c *Controller) SetMode(mode Mode, fb *framebuffer.Framebuffer) {
	ctrl1 := reg.Read(c.Base+offCtrl1) &^ (uint32(ctrlColorMask | ctrlCursor | ctrlOSD))
	if fb.Format == framebuffer.RGB565 {
		ctrl1 |= ctrlRGB565
	} else {
		ctrl1 |= ctrlXRGB8888
	}
	reg.Write(c.Base+offCtrl1, ctrl1)
	reg.Write(c.Base+offHoriz0, uint32(mode.HTotal-1)|uint32(mode.Width-1)<<16)
	reg.Write(c.Base+offHoriz1, uint32(mode.HSyncStart-1)|uint32(mode.HSyncEnd)<<16)
	reg.Write(c.Base+offVert0, uint32(mode.VTotal-1)|uint32(mode.Height-1)<<16)
	reg.Write(c.Base+offVert1, uint32(mode.VSyncStart)|uint32(mode.VSyncEnd)<<16)
	termCount := (mode.Width*fb.Format.Bpp()*8 + scanLineMax - 1) / scanLineMax
	reg.Write(c.Base+offOffset, uint32(fb.Stride)|uint32(termCount)<<16)
	reg.Write(c.Base+offThrod, defaultThreshold)
	c.SetAddress(fb.Addr)
}

// Enable turns on scanout and DAC/DP output path.
func (c *Controller) Enable() {
	reg.SetBits32(AST2700SCU0Base+offSCUDAC, 1<<11|1<<9)
	reg.SetBits32(AST2700SCU1Base+offSCU1DPDAC, 1<<10)
	TakeoverDP()
	reg.SetBits32(uintptr(c.Base+offCtrl1), ctrlEnable)
	// Write CTRL2 outright (not RMW): only bit0 (DAC power-on) is documented;
	// the BootMCU leaves garbage in the undocumented upper bits (e.g. 0xffeddd00,
	// where bits[31:20] act as VBLANK_LINE) which can blank the output. Our CRT
	// reset does not clear it, so set a clean value.
	reg.Write(c.Base+offCtrl2, ctrlDACEnable)
	reg.Write(c.Base+offCursor0, 0x3f3f)
	reg.Write(c.Base+offCursor1, 0x0fff1fff)
}

// Disable turns off scanout and DAC/DP output path.
func (c *Controller) Disable() {
	reg.ClearBits32(uintptr(c.Base+offCtrl1), ctrlEnable)
	reg.ClearBits32(uintptr(c.Base+offCtrl2), ctrlDACEnable)
}

// ConfigureAST2700DP800x600 configures AST2700 clocks and DP MCU for 800x600.
// The BootMCU is expected to have loaded and started the DPMCU firmware already.
func ConfigureAST2700DP800x600() {
	EnableAST2700Display()
	configureCRTClocks()
	reg.Write32(AST2700DPMCUBase+0xde0, dpResolution800)
}

// ConfigureCRT800x600 sets up the CRT pixel clock, performs a full CRT
// reset cycle, and programs the resolution.  Use this when the BootMCU has
// already set up VLink, loaded DPMCU firmware, and configured the DP
// handshake.
func ConfigureCRT800x600() {
	ConfigureCRTOnly800x600()
	reg.Write32(AST2700DPMCUBase+0xde0, dpResolution800)
}

// ConfigureCRTOnly800x600 sets up clocks and CRT reset without touching
// DPMCU DE0 or DP handshake.
func ConfigureCRTOnly800x600() {
	unlockSCU(AST2700SCU0Base)
	reg.Write(AST2700SCU0Base+offSCUClockStop, 1<<20)
	reg.Write(AST2700SCU0Base+offSCUReset1, 1<<13)
	lockSCU(AST2700SCU0Base)
	busyWait(10000)

	configureCRTClocks()

	unlockSCU(AST2700SCU0Base)
	reg.Write(AST2700SCU0Base+offSCUReset1+4, 1<<13)
	lockSCU(AST2700SCU0Base)
	busyWait(1000)

	unlockSCU(AST2700SCU0Base)
	reg.Write(AST2700SCU0Base+offSCUClockClr, 1<<20)
	reg.SetBits32(AST2700SCU0Base+offSCUDAC, 1<<11|1<<9)
	lockSCU(AST2700SCU0Base)
	busyWait(1000)
}

func busyWait(n int) {
	for i := 0; i < n; i++ {
		_ = i
	}
}

func configureCRTClocks() {
	unlockSCU(AST2700SCU0Base)
	reg.SetBits32(AST2700SCU0Base+offSCUClockSel2, 1<<14)
	reg.Write32(AST2700SCU0Base+0x340, 0x00190002)
	lockSCU(AST2700SCU0Base)
	unlockSCU(AST2700SCU1Base)
	reg.SetBits32(AST2700SCU1Base+offSCU1DPDAC, 1<<10)
	reg.Write32(AST2700SCU1Base+offSCU1DPCLK, dpPHYClockParam)
	lockSCU(AST2700SCU1Base)
}

// Dynamic-resolution CRT clock/timing registers.
const (
	offSCUXDClkSel  = 0x284      // SCU0 0x284 bit29 selects XDCLK (800/1000 MHz)
	offSCUCRT1CLK   = 0x340      // SCU0 0x340 CRT1CLK parameter (pixel clock)
	dpPHYClockParam = 0x1048000f // SCU1 0x320 DP PHY reference (link, not pixel clock)
)

// configureCRTClocksFor programs the CRT1CLK pixel clock for an arbitrary mode.
// The DP PHY reference (SCU1 0x320) is left at the fixed link-clock value; only
// CRT1CLK (SCU0 0x340) is resolution dependent. It returns false if the target
// pixel clock is not representable.
func configureCRTClocksFor(mode Mode) bool {
	xdclk := XDCLKkHz(reg.Read(AST2700SCU0Base + offSCUXDClkSel))
	param, _, _, ok := CRT1CLKParam(mode.PixelClockKHz, xdclk)
	if !ok {
		return false
	}

	unlockSCU(AST2700SCU0Base)
	reg.SetBits32(AST2700SCU0Base+offSCUClockSel2, 1<<14)
	reg.Write32(AST2700SCU0Base+offSCUCRT1CLK, param)
	lockSCU(AST2700SCU0Base)

	unlockSCU(AST2700SCU1Base)
	reg.SetBits32(AST2700SCU1Base+offSCU1DPDAC, 1<<10)
	reg.Write32(AST2700SCU1Base+offSCU1DPCLK, dpPHYClockParam)
	lockSCU(AST2700SCU1Base)
	return true
}

// ConfigureCRTOnly runs the CRT reset cycle and programs the pixel clock for an
// arbitrary mode, without touching the DPMCU DISPLAY_FORMAT word. It returns
// false if the mode's pixel clock cannot be represented.
func ConfigureCRTOnly(mode Mode) bool {
	unlockSCU(AST2700SCU0Base)
	reg.Write(AST2700SCU0Base+offSCUClockStop, 1<<20)
	reg.Write(AST2700SCU0Base+offSCUReset1, 1<<13)
	lockSCU(AST2700SCU0Base)
	busyWait(10000)

	if !configureCRTClocksFor(mode) {
		return false
	}

	unlockSCU(AST2700SCU0Base)
	reg.Write(AST2700SCU0Base+offSCUReset1+4, 1<<13)
	lockSCU(AST2700SCU0Base)
	busyWait(1000)

	unlockSCU(AST2700SCU0Base)
	reg.Write(AST2700SCU0Base+offSCUClockClr, 1<<20)
	reg.SetBits32(AST2700SCU0Base+offSCUDAC, 1<<11|1<<9)
	lockSCU(AST2700SCU0Base)
	busyWait(1000)
	return true
}

// ConfigureCRT runs the CRT reset cycle, programs the pixel clock, and writes
// the DPMCU DISPLAY_FORMAT word for an arbitrary mode. The BootMCU is expected
// to have loaded and started the DPMCU firmware already. It returns false if
// the mode's pixel clock cannot be represented.
func ConfigureCRT(mode Mode) bool {
	if !ConfigureCRTOnly(mode) {
		return false
	}
	reg.Write32(AST2700DPMCUBase+offMCUDE0, DisplayFormat(mode.ModeIndex))
	return true
}

// EnableAST2700Display enables AST2700 clocks, resets, VLink, and the DP scratch
// handshake. Call StartDPMCU() first (to release the DPMCU core running the
// staged firmware). It does NOT set DP_CONTROL_FROM_SOC yet; call TakeoverDP()
// as part of mode programming (Enable) so the GFX CRT drives the DP output.
func EnableAST2700Display() {
	unlockSCU(AST2700SCU0Base)
	reg.Write(AST2700SCU0Base+offSCUClockClr, 1<<3|1<<5|1<<17|1<<18|1<<20)
	reg.Write(AST2700SCU0Base+offSCUReset1+4, 1<<13|1<<28|1<<29)
	reg.Write(AST2700SCU0Base+offSCUReset2+4, 1<<12)
	reg.SetBits32(AST2700SCU0Base+offSCUDAC, 1<<11|1<<9)
	lockSCU(AST2700SCU0Base)

	unlockSCU(AST2700SCU1Base)
	reg.SetBits32(AST2700SCU1Base+offSCU1DPDAC, 1<<10)
	lockSCU(AST2700SCU1Base)

	initVLink()
	setDPScratchHandoff()
	reg.ClearBits32(AST2700DPBase+0xb8, dpControlFromSOC)
}

// TakeoverDP sets DP_CONTROL_FROM_SOC so the GFX CRT drives the DP output.
func TakeoverDP() {
	reg.SetBits32(AST2700DPBase+0xb8, dpControlFromSOC)
}

// WaitDPReady polls PCIE0/PCIE1 DP status for DPMCU link training completion.
func WaitDPReady(timeout time.Duration) bool {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		if dpReady(reg.Read(AST2700SCU0Base+offSCUPCIE0DP)) || dpReady(reg.Read(AST2700SCU0Base+offSCUPCIE1DP)) {
			return true
		}
		time.Sleep(100 * time.Millisecond)
	}
	return false
}

// DPMCURunning reports whether the DPMCU core has been released.
func DPMCURunning() bool {
	ctrl := reg.Read(AST2700DPMCUBase + offMCUCtrl)
	return ctrl&(mcuCtrlCoreSWRst|mcuCtrlAHBMSWRst) != 0
}

// StartDPMCU ensures the DisplayPort MCU core is running and its interrupts are
// enabled.
//
// On the AST2700 the BootMCU already brings up the DPMCU (loads firmware,
// releases the core, sets the DP scratch handshake) before releasing the CA35,
// so the core is normally already executing and link training by the time this
// runs — see the BootMCU display bring-up. Releasing the core late from the
// running CA35 races the CA35's live DRAM/fabric traffic and intermittently
// wedges it, which is why bring-up moved to the BootMCU. Re-asserting the
// (already-set) release bits here is an idempotent no-op; this remains as a
// belt-and-suspenders path for boot firmware that only staged the firmware.
func StartDPMCU() {
	ctrl := reg.Read32(AST2700DPMCUBase + offMCUCtrl)
	ctrl |= mcuCtrlCoreSWRst | mcuCtrlAHBMSWRst
	reg.Write32(AST2700DPMCUBase+offMCUCtrl, ctrl)
	reg.Write32(AST2700DPMCUBase+offMCUInt, mcuInterruptEnable)
}

// SetAddress changes the framebuffer scanout address.
func (c *Controller) SetAddress(addr uintptr) {
	reg.Write(c.Base+offAddr, AddressRegister(addr))
}

// AddressRegister returns the AST2700 GFX_ADDR register value for an A35
// virtual address.
//
// The GFX/CRT scanout is a direct DRAM (DARB) master that addresses DRAM with a
// DRAM-local, 0-based address. The CA35 sees DRAM at 0x400000000, so the
// framebuffer's DRAM offset is addr-0x400000000. GFX_ADDR (offset 0x80) holds
// framebuffer_addr[33:4] in bits[31:2], i.e. offset>>2.
//
// This matches the Linux aspeed-gfx driver, which writes dma_addr>>2 where
// dma_addr is the CA35 physical address: (0x400000000+off)>>2 = 0x100000000+off>>2,
// and the 0x100000000 (bit 32) is dropped by the 32-bit register write, leaving
// the DRAM-local off>>2.
func AddressRegister(addr uintptr) uint32 {
	offset := uint64(addr) - 0x400000000
	return uint32(offset >> 2)
}

// ClearVBlank clears a pending vertical blank interrupt status bit.
func (c *Controller) ClearVBlank() {
	reg.SetBits32(uintptr(c.Base+offCtrl1), ctrlVBlankSts)
}

// EnableVBlankInterrupt enables or disables the GFX vertical blank interrupt.
func (c *Controller) EnableVBlankInterrupt(enable bool) {
	c.ClearVBlank()
	if enable {
		reg.SetBits32(uintptr(c.Base+offCtrl1), ctrlVBlankEn)
		return
	}
	reg.ClearBits32(uintptr(c.Base+offCtrl1), ctrlVBlankEn)
	c.ClearVBlank()
}

// IRQPending reports whether the GFX vertical blank interrupt is pending.
func (c *Controller) IRQPending() bool {
	return reg.Read(c.Base+offCtrl1)&ctrlVBlankSts != 0
}

// HandleIRQ clears a pending GFX vertical blank interrupt.
func (c *Controller) HandleIRQ() bool {
	if !c.IRQPending() {
		return false
	}
	c.ClearVBlank()
	return true
}

// Status returns diagnostic GFX and DP register state.
func (c *Controller) Status() Status {
	pcie0 := reg.Read(AST2700SCU0Base + offSCUPCIE0DP)
	pcie1 := reg.Read(AST2700SCU0Base + offSCUPCIE1DP)
	return Status{
		Ctrl1:      reg.Read(c.Base + offCtrl1),
		Ctrl2:      reg.Read(c.Base + offCtrl2),
		CRTCStatus: reg.Read(c.Base + offStatus),
		Addr:       reg.Read(c.Base + offAddr),
		Offset:     reg.Read(c.Base + offOffset),
		Horiz0:     reg.Read(c.Base + offHoriz0),
		Horiz1:     reg.Read(c.Base + offHoriz1),
		Vert0:      reg.Read(c.Base + offVert0),
		Vert1:      reg.Read(c.Base + offVert1),
		DPSource:   reg.Read(AST2700DPBase + 0xb8),
		DPMCU:      reg.Read(AST2700DPMCUBase + 0xde0),
		DPMCUCtrl:  reg.Read(AST2700DPMCUBase + offMCUCtrl),
		DPMCUInt:   reg.Read(AST2700DPMCUBase + offMCUInt),
		DPVersion:  reg.Read(AST2700DPBase + 0x1c),
		ReDriver:   reg.Read(AST2700DPMCUBase + offMCUReDrv),
		SCUDAC:     reg.Read(AST2700SCU0Base + offSCUDAC),
		PCIE0DP:    pcie0,
		PCIE1DP:    pcie1,
		DPReady:    dpReady(pcie0) || dpReady(pcie1),
	}
}

// PrintStatus prints diagnostic GFX and DP register state.
func (c *Controller) PrintStatus(prefix string) {
	st := c.Status()
	fmt.Printf("%sGFX: ctrl1=%08x ctrl2=%08x status=%08x addr=%08x offset=%08x irq=%v\n", prefix, st.Ctrl1, st.Ctrl2, st.CRTCStatus, st.Addr, st.Offset, st.Ctrl1&ctrlVBlankSts != 0)
	fmt.Printf("%sGFX: horiz0=%08x horiz1=%08x vert0=%08x vert1=%08x\n", prefix, st.Horiz0, st.Horiz1, st.Vert0, st.Vert1)
	fmt.Printf("%sDP: version=%08x source=%08x dpmcu_de0=%08x mcu_ctrl=%08x mcu_int=%08x redrv=%08x\n", prefix, st.DPVersion, st.DPSource, st.DPMCU, st.DPMCUCtrl, st.DPMCUInt, st.ReDriver)
	fmt.Printf("%sDP: scu_dac=%08x pcie0=%08x(code=%02x) pcie1=%08x(code=%02x) ready=%v\n", prefix, st.SCUDAC, st.PCIE0DP, (st.PCIE0DP>>8)&0xff, st.PCIE1DP, (st.PCIE1DP>>8)&0xff, st.DPReady)
}

func dpReady(status uint32) bool {
	return status&dpReadyBit != 0
}

func unlockSCU(base uint32) {
	reg.Write(base+offSCUProtect, scuProtectKey)
}

func lockSCU(base uint32) {
	reg.Write(base+offSCUProtect, 1)
}

func initVLink() {
	reg.Write(packerCPU+0x10, 0x00030009)
	reg.Write(packerCPU+0x50, 0x10000000)
	reg.Write(packerCPU+0x44, 0x00100010)
	reg.Write(retimerCPU+0x10, 0x00030009)
	reg.Write(packerIO+0x10, 0x00030009)
	reg.Write(packerIO+0x44, 0x00010002)
	reg.Write(retimerIO+0x10, 0x00230009)
	reg.Write(retimerIO+0x44, 0x00100010)
}

// ReadDPMCUIMEM reads n words from DPMCU instruction memory by temporarily
// enabling AHB slave IMEM access.  This is only safe when the DPMCU is in a
// steady state (not being loaded).
func ReadDPMCUIMEM(off uint32, n int) []uint32 {
	ctrl := reg.Read32(AST2700DPMCUBase + offMCUCtrl)
	reg.Write32(AST2700DPMCUBase+offMCUCtrl, ctrl|mcuCtrlAHBSIMEMEn)
	words := make([]uint32, n)
	for i := range words {
		words[i] = reg.Read32(AST2700DPMCUBase + 0x20000 + uintptr(off) + uintptr(i)*4)
	}
	reg.Write32(AST2700DPMCUBase+offMCUCtrl, ctrl)
	return words
}

func setDPScratchHandoff() {
	for _, off := range []uint32{offSCUPCIE0DP, offSCUPCIE1DP} {
		v := reg.Read(AST2700SCU0Base + off)
		v &^= 0x7 << 9
		v |= 0x7 << 9
		// DP handshake bits the vendor VGA init sets alongside the [11:9]
		// handoff code (bit 7 and bit 12); required for the DPMCU to advance
		// past the host-programmed state to "executing".
		v |= (1 << 7) | (1 << 12)
		reg.Write(AST2700SCU0Base+off, v)
	}
}
