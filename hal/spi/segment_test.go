package spi

import "testing"

func TestSegmentDecodeDatasheetDefaults(t *testing.T) {
	tests := []struct {
		name      string
		soc       SoC
		regVal    uint32
		wantStart int64
		wantSize  int64
	}{
		// AST2700 CE0 reset value (SPI030 Init = 0x0ff00000): window
		// 0x0..0x0ff00000, 255 MiB, end exclusive.
		{"ast2700 CE0 default", AST2700, 0x0ff00000, 0, 0x0ff00000},
		// AST2700 disabled (start field == end field).
		{"ast2700 disabled", AST2700, 0x00000000, 0, 0},
		// AST2600 CE0 reset value (FMC30 Init = 0x07f00000): window
		// 0x0..0x08000000, 128 MiB, end inclusive (last unit + 1 MiB).
		{"ast2600 CE0 default", AST2600, 0x07f00000, 0, 0x08000000},
		{"ast2600 disabled", AST2600, 0x00000000, 0, 0},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			start, size, ok := SegmentDecode(tt.soc, tt.regVal)
			if !ok {
				t.Fatalf("SegmentDecode(%v, %#x) not ok", tt.soc, tt.regVal)
			}
			if start != tt.wantStart || size != tt.wantSize {
				t.Fatalf("SegmentDecode(%v, %#x) = (%#x, %#x), want (%#x, %#x)",
					tt.soc, tt.regVal, start, size, tt.wantStart, tt.wantSize)
			}
		})
	}
}

func TestSegmentEncodeDatasheetDefaults(t *testing.T) {
	tests := []struct {
		name    string
		soc     SoC
		start   int64
		size    int64
		wantReg uint32
	}{
		{"ast2700 CE0 default", AST2700, 0, 0x0ff00000, 0x0ff00000},
		{"ast2600 CE0 default", AST2600, 0, 0x08000000, 0x07f00000},
		{"ast2700 disabled", AST2700, 0, 0, 0x00000000},
		{"ast2600 disabled", AST2600, 0, 0, 0x00000000},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			reg, ok := SegmentEncode(tt.soc, tt.start, tt.size)
			if !ok {
				t.Fatalf("SegmentEncode(%v, %#x, %#x) not ok", tt.soc, tt.start, tt.size)
			}
			if reg != tt.wantReg {
				t.Fatalf("SegmentEncode(%v, %#x, %#x) = %#x, want %#x",
					tt.soc, tt.start, tt.size, reg, tt.wantReg)
			}
		})
	}
}

func TestSegmentRoundTrip(t *testing.T) {
	// Multi-unit ranges round-trip exactly on both generations.
	cases := []struct {
		soc         SoC
		start, size int64
	}{
		{AST2700, 0, 0x0400_0000},           // 0..64 MiB
		{AST2700, 0x0400_0000, 0x0400_0000}, // 64..128 MiB
		{AST2700, 0x0001_0000, 0x0002_0000}, // 64 KiB start, 128 KiB
		{AST2700, 0x1000_0000, 0x1000_0000}, // high window
		{AST2600, 0, 0x0800_0000},           // 0..128 MiB
		{AST2600, 0x0010_0000, 0x0030_0000}, // 1 MiB start, 3 MiB
		{AST2600, 0x0200_0000, 0x0200_0000}, // 32..64 MiB
	}
	for _, c := range cases {
		reg, ok := SegmentEncode(c.soc, c.start, c.size)
		if !ok {
			t.Fatalf("SegmentEncode(%v, %#x, %#x) not ok", c.soc, c.start, c.size)
		}
		start, size, ok := SegmentDecode(c.soc, reg)
		if !ok {
			t.Fatalf("SegmentDecode(%v, %#x) not ok", c.soc, reg)
		}
		if start != c.start || size != c.size {
			t.Fatalf("round-trip %v start=%#x size=%#x: reg=%#x decoded=(%#x,%#x)",
				c.soc, c.start, c.size, reg, start, size)
		}
	}
}

// TestSegmentAST2600SingleUnitLooksDisabled documents an inherent property of
// the AST2600 inclusive encoding: a segment exactly one unit (1 MiB) wide has
// its start and end fields land in the same unit, which is indistinguishable
// from a disabled segment. This is a hardware limitation, not a driver bug.
func TestSegmentAST2600SingleUnitLooksDisabled(t *testing.T) {
	reg, ok := SegmentEncode(AST2600, 0x0010_0000, 0x0010_0000)
	if !ok {
		t.Fatal("encode not ok")
	}
	_, size, ok := SegmentDecode(AST2600, reg)
	if !ok {
		t.Fatal("decode not ok")
	}
	if size != 0 {
		t.Fatalf("expected a single-unit AST2600 segment to decode as disabled (size 0), got %#x", size)
	}
}

func TestSegmentAlignmentAndValidation(t *testing.T) {
	if _, ok := SegmentEncode(SoCUnknown, 0, 0x1000); ok {
		t.Fatal("SoCUnknown should not be ok")
	}
	if _, _, ok := SegmentDecode(SoCUnknown, 0); ok {
		t.Fatal("SoCUnknown decode should not be ok")
	}
	// Misaligned start (not a multiple of the 64 KiB AST2700 unit).
	if _, ok := SegmentEncode(AST2700, 0x8000, 0x1_0000); ok {
		t.Fatal("misaligned start should be rejected")
	}
	// Misaligned size on AST2600 (not a multiple of 1 MiB).
	if _, ok := SegmentEncode(AST2600, 0, 0x8000); ok {
		t.Fatal("misaligned size should be rejected")
	}
	// Negative inputs.
	if _, ok := SegmentEncode(AST2700, -0x1_0000, 0x1_0000); ok {
		t.Fatal("negative start should be rejected")
	}
}
