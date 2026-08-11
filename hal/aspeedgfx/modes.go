package aspeedgfx

import "src.kyanite.computer/aspeed-go/hal/edid"

// ASTDP firmware video-format indices, consumed by the DPMCU through the
// DISPLAY_FORMAT word (DMEM 0xde0). These values match the ASPEED reference
// enumeration (Linux drm/ast ast_drv.h ASTDP_* defines); the DPMCU derives the
// Main Stream Attributes (link timing) for each index.
const (
	ASTDP_640x480_60   uint8 = 0x00
	ASTDP_800x600_60   uint8 = 0x05
	ASTDP_1024x768_60  uint8 = 0x09
	ASTDP_1280x1024_60 uint8 = 0x0D
	ASTDP_1600x1200_60 uint8 = 0x10
	ASTDP_1920x1200_60 uint8 = 0x14
	ASTDP_1920x1080_60 uint8 = 0x15
)

// DISPLAY_FORMAT field encoding (DMEM 0xde0). Cross-referenced with the AST2600
// astdp path (Linux drm/ast ast_dp.c), which writes the same three bytes to
// VGACRE0/E1/E2: MISC0 (0x20 = 24bpp), MISC1 (0x00) and the video format index.
// The AST2700 DPMCU packs them into one word with an enable byte in [31:24].
const (
	dfEnable uint32 = 0x01 << 24 // [31:24] enable region marker
	dfMISC1  uint32 = 0x00 << 8  // [15:8]  MSA MISC1
	dfMISC0  uint32 = 0x20       // [7:0]   MSA MISC0: 0x20 = 24bpp (8bpc RGB)
)

// modeTable holds the standard VESA DMT/CVT-RB timings for the ASTDP modes the
// DPMCU firmware supports. The GFX CRT timing must match what the DPMCU emits
// for the corresponding ModeIndex, so these are canonical standard timings (not
// raw EDID values). The 800x600 entry matches the known-good Mode800x600.
var modeTable = []Mode{
	{Width: 640, Height: 480, HTotal: 800, HSyncStart: 656, HSyncEnd: 752, VTotal: 525, VSyncStart: 490, VSyncEnd: 492, PixelClockKHz: 25175, ModeIndex: ASTDP_640x480_60},
	{Width: 800, Height: 600, HTotal: 1056, HSyncStart: 840, HSyncEnd: 968, VTotal: 628, VSyncStart: 601, VSyncEnd: 605, PixelClockKHz: 40000, ModeIndex: ASTDP_800x600_60},
	{Width: 1024, Height: 768, HTotal: 1344, HSyncStart: 1048, HSyncEnd: 1184, VTotal: 806, VSyncStart: 771, VSyncEnd: 777, PixelClockKHz: 65000, ModeIndex: ASTDP_1024x768_60},
	{Width: 1280, Height: 1024, HTotal: 1688, HSyncStart: 1328, HSyncEnd: 1440, VTotal: 1066, VSyncStart: 1025, VSyncEnd: 1028, PixelClockKHz: 108000, ModeIndex: ASTDP_1280x1024_60},
	{Width: 1600, Height: 1200, HTotal: 2160, HSyncStart: 1664, HSyncEnd: 1856, VTotal: 1250, VSyncStart: 1201, VSyncEnd: 1204, PixelClockKHz: 162000, ModeIndex: ASTDP_1600x1200_60},
	{Width: 1920, Height: 1080, HTotal: 2200, HSyncStart: 2008, HSyncEnd: 2052, VTotal: 1125, VSyncStart: 1084, VSyncEnd: 1089, PixelClockKHz: 148500, ModeIndex: ASTDP_1920x1080_60},
	// 1920x1200 uses CVT reduced blanking (the only 1920x1200 the DP can carry).
	{Width: 1920, Height: 1200, HTotal: 2080, HSyncStart: 1968, HSyncEnd: 2000, VTotal: 1235, VSyncStart: 1203, VSyncEnd: 1209, PixelClockKHz: 154000, ModeIndex: ASTDP_1920x1200_60},
}

// SupportedModes returns the table of display modes the driver can program,
// ordered from smallest to largest.
func SupportedModes() []Mode {
	out := make([]Mode, len(modeTable))
	copy(out, modeTable)
	return out
}

// ModeForResolution returns the canonical mode for an exact width/height, if
// the driver supports it.
func ModeForResolution(w, h int) (Mode, bool) {
	for _, m := range modeTable {
		if m.Width == w && m.Height == h {
			return m, true
		}
	}
	return Mode{}, false
}

// SelectMode chooses the best supported display mode for a parsed EDID. It
// prefers the display's native (preferred) timing; otherwise it returns the
// largest advertised resolution the driver supports. ok is false if the EDID
// advertises no supported mode.
func SelectMode(e *edid.EDID) (Mode, bool) {
	if e == nil {
		return Mode{}, false
	}
	if e.HasPreferred {
		if m, ok := ModeForResolution(e.Preferred.HActive, e.Preferred.VActive); ok {
			return m, true
		}
	}
	var best Mode
	found := false
	for _, r := range e.Resolutions {
		m, ok := ModeForResolution(r.W, r.H)
		if !ok {
			continue
		}
		if !found || m.Width*m.Height > best.Width*best.Height {
			best, found = m, true
		}
	}
	return best, found
}

// DisplayFormat returns the DMEM 0xde0 word for an ASTDP video-format index.
// For modeIndex 0x05 (800x600@60) this yields 0x01050020, matching the
// known-good value programmed by the vendor bring-up.
func DisplayFormat(modeIndex uint8) uint32 {
	return dfEnable | uint32(modeIndex)<<16 | dfMISC1 | dfMISC0
}

// XDCLK source frequencies in kHz (datasheet SCU0 0x340 note).
const (
	xdclk1000kHz = 1000000
	xdclk800kHz  = 800000
)

// XDCLKkHz returns the CRT1CLK source frequency (XDCLK) in kHz, selected by
// SCU0 0x284 bit29: 0 -> 1000 MHz, 1 -> 800 MHz (datasheet SCU0 0x340 note).
func XDCLKkHz(scu0Reg284 uint32) int {
	if scu0Reg284&(1<<29) != 0 {
		return xdclk800kHz
	}
	return xdclk1000kHz
}

const crtClkRNMax = 0xFFFF // R and N are 16-bit fields in SCU0 0x340

// CRT1CLKParam computes the SCU0 0x340 CRT1CLK parameter word for a target
// pixel clock. The datasheet defines CRTCLK = 1/2 * XDCLK * R / N with
// R = bits[15:0], N = bits[31:16] and the constraint N >= R.
//
// It searches for the R/N pair (both in 1..65535, N >= R) whose resulting clock
// is closest to pixelKHz. ok is false if no representable value exists (e.g. a
// target above XDCLK/2). The returned r and n are exposed for diagnostics.
func CRT1CLKParam(pixelKHz, xdclkKHz int) (param uint32, r, n int, ok bool) {
	if pixelKHz <= 0 || xdclkKHz <= 0 {
		return 0, 0, 0, false
	}
	// CRTCLK = (xdclk/2) * R/N  =>  R/N = pixelKHz / (xdclk/2).
	half := xdclkKHz / 2
	if pixelKHz > half {
		return 0, 0, 0, false // exceeds XDCLK/2, not representable (N>=R)
	}

	bestErr := -1
	for nn := 1; nn <= crtClkRNMax; nn++ {
		// rr = round(pixelKHz * nn / half)
		rr := (pixelKHz*nn + half/2) / half
		if rr < 1 || rr > nn || rr > crtClkRNMax {
			continue
		}
		got := half * rr / nn
		err := got - pixelKHz
		if err < 0 {
			err = -err
		}
		if bestErr < 0 || err < bestErr {
			bestErr, r, n = err, rr, nn
			if err == 0 {
				break
			}
		}
	}
	if bestErr < 0 {
		return 0, 0, 0, false
	}
	return uint32(n)<<16 | uint32(r), r, n, true
}
