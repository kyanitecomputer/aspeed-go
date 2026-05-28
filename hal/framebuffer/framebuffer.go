// Package framebuffer provides a bare-metal linear framebuffer abstraction.
package framebuffer

import (
	"encoding/binary"
	"unsafe"
)

// PixelFormat describes the framebuffer color layout.
type PixelFormat uint8

const (
	// XRGB8888 is 32bpp with ignored alpha in bits [31:24].
	XRGB8888 PixelFormat = iota
	// RGB565 is 16bpp red/green/blue.
	RGB565
)

// Bpp returns bytes per pixel.
func (pf PixelFormat) Bpp() int {
	switch pf {
	case RGB565:
		return 2
	default:
		return 4
	}
}

// Color is a 32-bit RGB color value.
type Color uint32

// RGB returns an opaque RGB color.
func RGB(r, g, b uint8) Color {
	return Color(uint32(r)<<16 | uint32(g)<<8 | uint32(b))
}

// Pack converts c to the target pixel format.
func (c Color) Pack(pf PixelFormat) uint32 {
	r := uint8(c >> 16)
	g := uint8(c >> 8)
	b := uint8(c)
	if pf == RGB565 {
		return uint32(r>>3)<<11 | uint32(g>>2)<<5 | uint32(b>>3)
	}
	return uint32(c)
}

// Framebuffer represents a linear pixel buffer.
type Framebuffer struct {
	Addr   uintptr
	Buf    []byte
	Width  int
	Height int
	Stride int
	Format PixelFormat
}

// Config describes a framebuffer allocation.
type Config struct {
	Width    int
	Height   int
	Format   PixelFormat
	PhysAddr uintptr
	Stride   int
}

// New creates a framebuffer view over a caller-provided DMA-visible buffer.
func New(cfg Config) *Framebuffer {
	stride := cfg.Stride
	if stride == 0 {
		stride = cfg.Width * cfg.Format.Bpp()
	}
	return &Framebuffer{
		Addr:   cfg.PhysAddr,
		Buf:    unsafe.Slice((*byte)(unsafe.Pointer(cfg.PhysAddr)), stride*cfg.Height),
		Width:  cfg.Width,
		Height: cfg.Height,
		Stride: stride,
		Format: cfg.Format,
	}
}

// SetPixel writes one clipped pixel.
func (fb *Framebuffer) SetPixel(x, y int, c Color) {
	if x < 0 || y < 0 || x >= fb.Width || y >= fb.Height {
		return
	}
	off := y*fb.Stride + x*fb.Format.Bpp()
	packed := c.Pack(fb.Format)
	if fb.Format == RGB565 {
		binary.LittleEndian.PutUint16(fb.Buf[off:], uint16(packed))
		return
	}
	binary.LittleEndian.PutUint32(fb.Buf[off:], packed)
}

// Fill fills the framebuffer with a solid color.
func (fb *Framebuffer) Fill(c Color) {
	fb.FillRect(0, 0, fb.Width, fb.Height, c)
}

// FillRect fills a clipped rectangle.
func (fb *Framebuffer) FillRect(x, y, w, h int, c Color) {
	if x < 0 {
		w += x
		x = 0
	}
	if y < 0 {
		h += y
		y = 0
	}
	if x+w > fb.Width {
		w = fb.Width - x
	}
	if y+h > fb.Height {
		h = fb.Height - y
	}
	if w <= 0 || h <= 0 {
		return
	}
	packed := c.Pack(fb.Format)
	bpp := fb.Format.Bpp()
	rowBytes := w * bpp
	for yy := y; yy < y+h; yy++ {
		row := fb.Buf[yy*fb.Stride+x*bpp:][:rowBytes]
		if fb.Format == RGB565 {
			v := uint16(packed)
			for off := 0; off < len(row); off += 2 {
				binary.LittleEndian.PutUint16(row[off:], v)
			}
			continue
		}
		for off := 0; off < len(row); off += 4 {
			binary.LittleEndian.PutUint32(row[off:], packed)
		}
	}
}

// HLine draws a clipped horizontal line.
func (fb *Framebuffer) HLine(x0, x1, y int, c Color) {
	if x0 > x1 {
		x0, x1 = x1, x0
	}
	fb.FillRect(x0, y, x1-x0+1, 1, c)
}

// VLine draws a clipped vertical line.
func (fb *Framebuffer) VLine(x, y0, y1 int, c Color) {
	if y0 > y1 {
		y0, y1 = y1, y0
	}
	fb.FillRect(x, y0, 1, y1-y0+1, c)
}
