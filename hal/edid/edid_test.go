package edid

import "testing"

// buildEDID assembles a valid 128-byte EDID base block with a single preferred
// detailed timing descriptor and a fixed-up checksum.
func buildEDID(dtd [18]byte, established [3]byte, std [16]byte) []byte {
	b := make([]byte, BlockSize)
	copy(b, edidHeader[:])
	// Manufacturer "DEL" -> packed big-endian.
	b[8] = 0x10
	b[9] = 0xAC
	b[10] = 0x34
	b[11] = 0x12
	b[35], b[36], b[37] = established[0], established[1], established[2]
	copy(b[38:54], std[:])
	copy(b[54:72], dtd[:])
	b[126] = 0 // no extensions
	var sum byte
	for i := 0; i < BlockSize-1; i++ {
		sum += b[i]
	}
	b[127] = byte(-int8(sum)) // make total wrap to 0
	return b
}

// dt1080p is a 1920x1080@60 detailed timing (pixel clock 148.5 MHz).
// HActive 1920, HBlank 280, HFront 88, HSync 44.
// VActive 1080, VBlank 45, VFront 4, VSync 5.
func dt1080p() [18]byte {
	var d [18]byte
	pc := 14850 // 148.5 MHz in 10 kHz units
	d[0] = byte(pc & 0xFF)
	d[1] = byte(pc >> 8)
	d[2] = 1920 & 0xFF
	d[3] = 280 & 0xFF
	d[4] = byte((1920>>8)<<4) | byte(280>>8)
	d[5] = 1080 & 0xFF
	d[6] = 45 & 0xFF
	d[7] = byte((1080>>8)<<4) | byte(45>>8)
	d[8] = 88 & 0xFF
	d[9] = 44 & 0xFF
	d[10] = byte((4&0x0F)<<4) | byte(5&0x0F)
	d[11] = 0
	return d
}

func TestParsePreferred1080p(t *testing.T) {
	b := buildEDID(dt1080p(), [3]byte{}, [16]byte{0x01, 0x01})
	e, err := Parse(b)
	if err != nil {
		t.Fatalf("Parse: %v", err)
	}
	if !e.HasPreferred {
		t.Fatal("expected preferred timing")
	}
	p := e.Preferred
	checks := []struct {
		name string
		got  int
		want int
	}{
		{"PixelClockKHz", p.PixelClockKHz, 148500},
		{"HActive", p.HActive, 1920},
		{"HBlank", p.HBlank, 280},
		{"HFrontPorch", p.HFrontPorch, 88},
		{"HSyncWidth", p.HSyncWidth, 44},
		{"VActive", p.VActive, 1080},
		{"VBlank", p.VBlank, 45},
		{"VFrontPorch", p.VFrontPorch, 4},
		{"VSyncWidth", p.VSyncWidth, 5},
		{"HTotal", p.HTotal(), 2200},
		{"VTotal", p.VTotal(), 1125},
		{"HSyncStart", p.HSyncStart(), 2008},
		{"HSyncEnd", p.HSyncEnd(), 2052},
		{"VSyncStart", p.VSyncStart(), 1084},
		{"VSyncEnd", p.VSyncEnd(), 1089},
	}
	for _, c := range checks {
		if c.got != c.want {
			t.Errorf("%s = %d, want %d", c.name, c.got, c.want)
		}
	}
	if e.ManufacturerID != "DEL" {
		t.Errorf("ManufacturerID = %q, want DEL", e.ManufacturerID)
	}
	if len(e.Resolutions) == 0 || e.Resolutions[0] != (Resolution{1920, 1080}) {
		t.Errorf("Resolutions[0] = %v, want 1920x1080 first", e.Resolutions)
	}
}

func TestParseHeaderAndChecksum(t *testing.T) {
	b := buildEDID(dt1080p(), [3]byte{}, [16]byte{0x01, 0x01})

	bad := append([]byte(nil), b...)
	bad[0] = 0x11
	if _, err := Parse(bad); err != ErrHeader {
		t.Errorf("bad header: got %v, want ErrHeader", err)
	}

	bad = append([]byte(nil), b...)
	bad[127] ^= 0xFF
	if _, err := Parse(bad); err != ErrChecksum {
		t.Errorf("bad checksum: got %v, want ErrChecksum", err)
	}

	if _, err := Parse(b[:64]); err != ErrShort {
		t.Errorf("short: got %v, want ErrShort", err)
	}
}

func TestEstablishedAndStandardTimings(t *testing.T) {
	// Established: 640x480@60 (b0 bit5) and 1024x768@60 (b1 bit3).
	est := [3]byte{0x20, 0x08, 0x00}
	// Standard timing: 1280x1024 (4:3 encoding gives 960; use 5:4 -> 1024).
	// horizontal = (b0+31)*8 = 1280 -> b0 = 129; aspect 5:4 -> b1 bits[7:6]=10.
	var std [16]byte
	std[0] = 129
	std[1] = 0x80 // 5:4
	for i := 2; i < 16; i++ {
		std[i] = 0x01
	}
	b := buildEDID(dt1080p(), est, std)
	e, err := Parse(b)
	if err != nil {
		t.Fatalf("Parse: %v", err)
	}
	want := map[Resolution]bool{
		{1920, 1080}: true,
		{640, 480}:   true,
		{1024, 768}:  true,
		{1280, 1024}: true,
	}
	got := map[Resolution]bool{}
	for _, r := range e.Resolutions {
		got[r] = true
	}
	for r := range want {
		if !got[r] {
			t.Errorf("missing resolution %v in %v", r, e.Resolutions)
		}
	}
}
