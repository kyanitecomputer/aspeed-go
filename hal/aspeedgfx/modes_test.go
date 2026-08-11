package aspeedgfx

import (
	"testing"

	"src.kyanite.computer/aspeed-go/hal/edid"
)

func TestSelectMode(t *testing.T) {
	// Preferred 1920x1080 is supported -> selected directly.
	e := &edid.EDID{
		HasPreferred: true,
		Preferred:    edid.DetailedTiming{HActive: 1920, VActive: 1080},
		Resolutions:  []edid.Resolution{{W: 1920, H: 1080}, {W: 1024, H: 768}},
	}
	if m, ok := SelectMode(e); !ok || m.Width != 1920 || m.Height != 1080 {
		t.Errorf("preferred: got %dx%d ok=%v, want 1920x1080", m.Width, m.Height, ok)
	}

	// Preferred unsupported -> largest supported advertised resolution.
	e = &edid.EDID{
		HasPreferred: true,
		Preferred:    edid.DetailedTiming{HActive: 1366, VActive: 768},
		Resolutions:  []edid.Resolution{{W: 1366, H: 768}, {W: 800, H: 600}, {W: 1280, H: 1024}},
	}
	if m, ok := SelectMode(e); !ok || m.Width != 1280 || m.Height != 1024 {
		t.Errorf("fallback: got %dx%d ok=%v, want 1280x1024", m.Width, m.Height, ok)
	}

	// No supported mode advertised.
	e = &edid.EDID{Resolutions: []edid.Resolution{{W: 1366, H: 768}}}
	if _, ok := SelectMode(e); ok {
		t.Error("expected no supported mode")
	}

	if _, ok := SelectMode(nil); ok {
		t.Error("nil EDID should return false")
	}
}

func TestDisplayFormat(t *testing.T) {
	// 800x600@60 must reproduce the known-good vendor value.
	if got := DisplayFormat(ASTDP_800x600_60); got != 0x01050020 {
		t.Errorf("DisplayFormat(800x600) = %#08x, want 0x01050020", got)
	}
	if got := DisplayFormat(ASTDP_1920x1200_60); got != 0x01140020 {
		t.Errorf("DisplayFormat(1920x1200) = %#08x, want 0x01140020", got)
	}
}

func TestXDCLKkHz(t *testing.T) {
	if got := XDCLKkHz(0); got != 1000000 {
		t.Errorf("XDCLKkHz(bit29=0) = %d, want 1000000", got)
	}
	if got := XDCLKkHz(1 << 29); got != 800000 {
		t.Errorf("XDCLKkHz(bit29=1) = %d, want 800000", got)
	}
}

func TestCRT1CLKParam800x600Matches(t *testing.T) {
	// The known-good 800x600 value is R=2, N=25 -> 0x00190002 at XDCLK=1000MHz.
	param, r, n, ok := CRT1CLKParam(40000, 1000000)
	if !ok {
		t.Fatal("CRT1CLKParam not ok")
	}
	if r != 2 || n != 25 {
		t.Errorf("R/N = %d/%d, want 2/25", r, n)
	}
	if param != 0x00190002 {
		t.Errorf("param = %#08x, want 0x00190002", param)
	}
}

func TestCRT1CLKParamExactness(t *testing.T) {
	tests := []struct {
		name    string
		pixel   int
		xdclk   int
		wantErr int // max tolerated kHz error
	}{
		{"640x480 25.175MHz", 25175, 1000000, 50},
		{"1024x768 65MHz", 65000, 1000000, 1},
		{"1280x1024 108MHz", 108000, 1000000, 1},
		{"1600x1200 162MHz", 162000, 1000000, 1},
		{"1920x1080 148.5MHz", 148500, 1000000, 1},
		{"1920x1200 154MHz", 154000, 1000000, 1},
		{"148.5MHz at XDCLK 800", 148500, 800000, 1},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			param, r, n, ok := CRT1CLKParam(tc.pixel, tc.xdclk)
			if !ok {
				t.Fatal("not ok")
			}
			if n < r {
				t.Errorf("constraint N>=R violated: R=%d N=%d", r, n)
			}
			if r < 1 || r > 0xFFFF || n < 1 || n > 0xFFFF {
				t.Errorf("R/N out of range: R=%d N=%d", r, n)
			}
			got := (tc.xdclk / 2) * r / n
			err := got - tc.pixel
			if err < 0 {
				err = -err
			}
			if err > tc.wantErr {
				t.Errorf("clock = %d kHz (R=%d N=%d param=%#08x), want ~%d (err %d > %d)",
					got, r, n, param, tc.pixel, err, tc.wantErr)
			}
		})
	}
}

func TestCRT1CLKParamRejectsTooHigh(t *testing.T) {
	// Above XDCLK/2 is not representable with N>=R.
	if _, _, _, ok := CRT1CLKParam(600000, 1000000); ok {
		t.Error("expected not-ok for pixel clock above XDCLK/2")
	}
}

func TestModeForResolution(t *testing.T) {
	m, ok := ModeForResolution(1920, 1080)
	if !ok {
		t.Fatal("1920x1080 not found")
	}
	if m.ModeIndex != ASTDP_1920x1080_60 || m.PixelClockKHz != 148500 {
		t.Errorf("unexpected mode %+v", m)
	}
	if _, ok := ModeForResolution(1234, 567); ok {
		t.Error("unexpected match for unsupported resolution")
	}
}
