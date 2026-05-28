// Package edid parses VESA E-EDID base blocks to discover a display's
// supported and preferred timings.
//
// The parser is intentionally self-contained (no hardware access) so it can be
// unit-tested on the host. It understands the EDID 1.3/1.4 base block: the
// header/checksum, the preferred detailed timing descriptor, and the
// established/standard timing lists used for mode discovery.
package edid

import (
	"errors"
	"fmt"
)

// BlockSize is the length in bytes of one EDID block.
const BlockSize = 128

var (
	// ErrShort is returned when the input is smaller than one EDID block.
	ErrShort = errors.New("edid: data shorter than one 128-byte block")
	// ErrHeader is returned when the fixed EDID header is missing.
	ErrHeader = errors.New("edid: bad header signature")
	// ErrChecksum is returned when the block checksum does not sum to zero.
	ErrChecksum = errors.New("edid: bad checksum")
)

// DetailedTiming describes a single detailed timing descriptor in pixels.
//
// Horizontal/vertical sync timings are expressed as front porch (sync offset),
// sync pulse width, and blanking, matching the EDID encoding. Derived totals
// are available via the helper methods.
type DetailedTiming struct {
	PixelClockKHz int

	HActive     int
	HBlank      int
	HFrontPorch int // horizontal sync offset
	HSyncWidth  int

	VActive     int
	VBlank      int
	VFrontPorch int // vertical sync offset
	VSyncWidth  int

	Interlaced bool
}

// HTotal returns the total horizontal pixels (active + blanking).
func (t DetailedTiming) HTotal() int { return t.HActive + t.HBlank }

// VTotal returns the total vertical lines (active + blanking).
func (t DetailedTiming) VTotal() int { return t.VActive + t.VBlank }

// HSyncStart returns the horizontal sync start pixel.
func (t DetailedTiming) HSyncStart() int { return t.HActive + t.HFrontPorch }

// HSyncEnd returns the horizontal sync end pixel.
func (t DetailedTiming) HSyncEnd() int { return t.HActive + t.HFrontPorch + t.HSyncWidth }

// VSyncStart returns the vertical sync start line.
func (t DetailedTiming) VSyncStart() int { return t.VActive + t.VFrontPorch }

// VSyncEnd returns the vertical sync end line.
func (t DetailedTiming) VSyncEnd() int { return t.VActive + t.VFrontPorch + t.VSyncWidth }

// Resolution is a display resolution in pixels.
type Resolution struct {
	W, H int
}

// EDID is the parsed content of an EDID base block.
type EDID struct {
	// ManufacturerID is the 3-letter PNP vendor ID (e.g. "DEL").
	ManufacturerID string
	// ProductCode is the manufacturer product code.
	ProductCode uint16
	// Extensions is the number of extension blocks that follow.
	Extensions int
	// Preferred is the preferred detailed timing (first DTD). It is the
	// display's native timing and is the value mode selection should favor.
	Preferred DetailedTiming
	// HasPreferred reports whether a valid preferred DTD was found.
	HasPreferred bool
	// Resolutions lists distinct resolutions advertised across the detailed,
	// established and standard timing sections, most-preferred first.
	Resolutions []Resolution
}

var edidHeader = [8]byte{0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00}

// Parse parses the EDID base block at the start of b. Any extension blocks are
// ignored. The first block's header and checksum are validated.
func Parse(b []byte) (*EDID, error) {
	if len(b) < BlockSize {
		return nil, ErrShort
	}
	for i := 0; i < len(edidHeader); i++ {
		if b[i] != edidHeader[i] {
			return nil, ErrHeader
		}
	}
	var sum byte
	for i := 0; i < BlockSize; i++ {
		sum += b[i]
	}
	if sum != 0 {
		return nil, ErrChecksum
	}

	e := &EDID{
		ManufacturerID: decodeManufacturer(b[8], b[9]),
		ProductCode:    uint16(b[10]) | uint16(b[11])<<8,
		Extensions:     int(b[126]),
	}

	seen := map[Resolution]bool{}
	addRes := func(w, h int) {
		if w <= 0 || h <= 0 {
			return
		}
		r := Resolution{w, h}
		if !seen[r] {
			seen[r] = true
			e.Resolutions = append(e.Resolutions, r)
		}
	}

	// Detailed timing descriptors occupy bytes 54..125 (four 18-byte slots).
	for i := 0; i < 4; i++ {
		off := 54 + i*18
		d := b[off : off+18]
		if d[0] == 0 && d[1] == 0 {
			continue // monitor descriptor, not a timing
		}
		t := parseDTD(d)
		if !e.HasPreferred {
			e.Preferred = t
			e.HasPreferred = true
		}
		addRes(t.HActive, t.VActive)
	}

	for _, r := range establishedTimings(b[35], b[36], b[37]) {
		addRes(r.W, r.H)
	}
	for i := 0; i < 8; i++ {
		off := 38 + i*2
		if r, ok := parseStandardTiming(b[off], b[off+1]); ok {
			addRes(r.W, r.H)
		}
	}

	return e, nil
}

// parseDTD decodes an 18-byte detailed timing descriptor.
func parseDTD(d []byte) DetailedTiming {
	pixClk := (int(d[1])<<8 | int(d[0])) * 10 // 10 kHz units -> kHz

	hActive := int(d[2]) | (int(d[4]&0xF0) << 4)
	hBlank := int(d[3]) | (int(d[4]&0x0F) << 8)
	vActive := int(d[5]) | (int(d[7]&0xF0) << 4)
	vBlank := int(d[6]) | (int(d[7]&0x0F) << 8)

	hFront := int(d[8]) | (int(d[11]&0xC0) << 2)
	hSync := int(d[9]) | (int(d[11]&0x30) << 4)
	vFront := int(d[10]>>4) | (int(d[11]&0x0C) << 2)
	vSync := int(d[10]&0x0F) | (int(d[11]&0x03) << 4)

	return DetailedTiming{
		PixelClockKHz: pixClk,
		HActive:       hActive,
		HBlank:        hBlank,
		HFrontPorch:   hFront,
		HSyncWidth:    hSync,
		VActive:       vActive,
		VBlank:        vBlank,
		VFrontPorch:   vFront,
		VSyncWidth:    vSync,
		Interlaced:    d[17]&0x80 != 0,
	}
}

// parseStandardTiming decodes a 2-byte standard timing identifier. An unused
// slot is encoded as 0x01 0x01.
func parseStandardTiming(b0, b1 byte) (Resolution, bool) {
	if b0 == 0x01 && b1 == 0x01 {
		return Resolution{}, false
	}
	if b0 == 0 {
		return Resolution{}, false
	}
	w := (int(b0) + 31) * 8
	var h int
	switch (b1 >> 6) & 0x3 {
	case 0: // 16:10
		h = w * 10 / 16
	case 1: // 4:3
		h = w * 3 / 4
	case 2: // 5:4
		h = w * 4 / 5
	case 3: // 16:9
		h = w * 9 / 16
	}
	return Resolution{w, h}, true
}

// establishedTimings decodes the three established-timing bitmap bytes (EDID
// offsets 35..37) into resolutions.
func establishedTimings(b0, b1, b2 byte) []Resolution {
	type bit struct {
		mask byte
		res  Resolution
	}
	var out []Resolution
	group := func(v byte, bits []bit) {
		for _, b := range bits {
			if v&b.mask != 0 {
				out = append(out, b.res)
			}
		}
	}
	group(b0, []bit{
		{0x80, Resolution{720, 400}},
		{0x40, Resolution{720, 400}},
		{0x20, Resolution{640, 480}},
		{0x10, Resolution{640, 480}},
		{0x08, Resolution{640, 480}},
		{0x04, Resolution{640, 480}},
		{0x02, Resolution{800, 600}},
		{0x01, Resolution{800, 600}},
	})
	group(b1, []bit{
		{0x80, Resolution{800, 600}},
		{0x40, Resolution{800, 600}},
		{0x20, Resolution{832, 624}},
		{0x10, Resolution{1024, 768}},
		{0x08, Resolution{1024, 768}},
		{0x04, Resolution{1024, 768}},
		{0x02, Resolution{1024, 768}},
		{0x01, Resolution{1280, 1024}},
	})
	group(b2, []bit{
		{0x80, Resolution{1152, 870}},
	})
	return out
}

// decodeManufacturer decodes the 2-byte big-endian packed PNP vendor ID.
func decodeManufacturer(b0, b1 byte) string {
	v := uint16(b0)<<8 | uint16(b1)
	c1 := byte((v>>10)&0x1F) + 'A' - 1
	c2 := byte((v>>5)&0x1F) + 'A' - 1
	c3 := byte(v&0x1F) + 'A' - 1
	if !isUpper(c1) || !isUpper(c2) || !isUpper(c3) {
		return fmt.Sprintf("%04X", v)
	}
	return string([]byte{c1, c2, c3})
}

func isUpper(c byte) bool { return c >= 'A' && c <= 'Z' }
