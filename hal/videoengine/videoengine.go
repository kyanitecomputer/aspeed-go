// Package videoengine provides an ASPEED video capture/compression driver.
package videoengine

import (
	"errors"
	"time"
	"unsafe"

	"src.kyanite.computer/aspeed-go/reg"
)

const (
	AST2700Video0Base = 0x120a0000
	AST2700Video1Base = 0x120a1000

	offProtectionKey        = 0x000
	offSeqCtrl              = 0x004
	offCtrl                 = 0x008
	offTGS0                 = 0x00c
	offTGS1                 = 0x010
	offScalingFactor        = 0x014
	offScalingFilter0       = 0x018
	offScalingFilter1       = 0x01c
	offScalingFilter2       = 0x020
	offScalingFilter3       = 0x024
	offBCDControl           = 0x02c
	offCaptureWindow        = 0x030
	offCompressWindow       = 0x034
	offCompressProcOffset   = 0x038
	offCompressOffset       = 0x03c
	offJPEGAddr             = 0x040
	offSource0Addr          = 0x044
	offSourceScanlineOffset = 0x048
	offSource1Addr          = 0x04c
	offBCDAddr              = 0x050
	offCompressAddr         = 0x054
	offStreamBufSize        = 0x058
	offCompressCtrl         = 0x060
	offCBAddr               = 0x06c
	offCompressSizeReadback = 0x084
	offSourceLREdgeDetect   = 0x090
	offSourceTBEdgeDetect   = 0x094
	offModeDetectStatus     = 0x098
	offSyncStatus           = 0x09c
	offHTotalPixels         = 0x0a0
	offInterruptCtrl        = 0x304
	offInterruptStatus      = 0x308
	offModeDetect           = 0x30c
	offMemRestrictStart     = 0x310
	offMemRestrictEnd       = 0x314

	protectionKeyUnlock = 0x1a038aa8

	seqTrigModeDetect = 1 << 0
	seqTrigCapture    = 1 << 1
	seqForceIdle      = 1 << 2
	seqTrigCompress   = 1 << 4
	seqAutoCompress   = 1 << 5
	seqYUV420         = 1 << 10
	seqJPEGMode       = 1 << 13
	seqTrigJPEG       = 1 << 15
	seqCaptureBusy    = 1 << 16
	seqCompressBusy   = 1 << 18

	ctrlSource      = 1 << 2
	ctrlDirectFetch = 1 << 5
	ctrlCaptureRGB  = 2 << 6
	ctrlCompareOnly = 1 << 31

	intModeDetect      = 1 << 4
	intCaptureComplete = 1 << 1
	intCompressReady   = 1 << 2
	intCompressDone    = 1 << 3
	intFrameComplete   = 1 << 5
	intErrorMask       = 1<<0 | 1<<6 | 1<<9
	intAll             = 1<<0 | 1<<1 | 1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<8 | 1<<9 | 1<<10 | 1<<11
)

var ErrTimeout = errors.New("videoengine: timeout")

// Input selects the video source.
type Input uint8

const (
	InputHostVGA Input = iota
	InputGraphicsCRT
	InputExternalADC
	InputExternalDigital
)

// CaptureFormat selects the uncompressed source format.
type CaptureFormat uint8

const (
	CaptureYUVStudio CaptureFormat = iota
	CaptureYUVFull
	CaptureRGB
	CaptureGray
)

// CompressFormat selects the compressed output format.
type CompressFormat uint8

const (
	CompressASPEED CompressFormat = iota
	CompressJPEG
)

// Geometry describes a detected video mode.
type Geometry struct {
	Width     int
	Height    int
	HPeriod   int
	Interlace bool
	NoSignal  bool
}

// Buffers contains DMA addresses used by the video engine.
type Buffers struct {
	Source0  uint64
	Source1  uint64
	JPEG     uint64
	BCD      uint64
	Compress uint64
	Cursor   uint64
	Size     uint32
}

// BufferMemory contains CPU slices used as video engine DMA buffers.
type BufferMemory struct {
	Source0  []byte
	Source1  []byte
	JPEG     []byte
	BCD      []byte
	Compress []byte
	Cursor   []byte
}

// Config configures capture/compression operation.
type Config struct {
	Input          Input
	CaptureFormat  CaptureFormat
	CompressFormat CompressFormat
	YUV420         bool
	DirectFetch    bool
	CompareOnly    bool
	Width          int
	Height         int
	Stride         int
	Buffers        Buffers
}

// Engine is an ASPEED video engine instance.
type Engine struct {
	Base uint32
}

// DMA returns hardware addresses and stream size for preallocated buffers.
func DMA(mem BufferMemory) Buffers {
	return Buffers{
		Source0:  addr(mem.Source0),
		Source1:  addr(mem.Source1),
		JPEG:     addr(mem.JPEG),
		BCD:      addr(mem.BCD),
		Compress: addr(mem.Compress),
		Cursor:   addr(mem.Cursor),
		Size:     uint32(len(mem.Compress)),
	}
}

// InitForCapture resets the engine and applies a capture configuration.
func (e *Engine) InitForCapture(cfg Config) {
	e.Reset()
	e.Configure(cfg)
	e.AckInterrupts()
}

// Unlock unlocks protected engine registers.
func (e *Engine) Unlock() {
	reg.Write(e.Base+offProtectionKey, protectionKeyUnlock)
}

// Reset forces the sequencer idle and clears interrupts.
func (e *Engine) Reset() {
	e.Unlock()
	reg.Write(e.Base+offSeqCtrl, seqForceIdle)
	reg.Write(e.Base+offInterruptStatus, intAll)
}

// Configure programs capture and compression buffers.
func (e *Engine) Configure(cfg Config) {
	e.Unlock()
	ctrl := uint32(cfg.CaptureFormat) << 6
	if cfg.Input == InputGraphicsCRT || cfg.Input == InputExternalDigital {
		ctrl |= ctrlSource
	}
	if cfg.DirectFetch {
		ctrl |= ctrlDirectFetch
	}
	if cfg.CompareOnly {
		ctrl |= ctrlCompareOnly
	}
	reg.Write(e.Base+offCtrl, ctrl)
	reg.Write(e.Base+offCaptureWindow, window(cfg.Width, cfg.Height))
	reg.Write(e.Base+offCompressWindow, window(cfg.Width, cfg.Height))
	reg.Write(e.Base+offSourceScanlineOffset, uint32(cfg.Stride))
	reg.Write(e.Base+offSource0Addr, makeAddr(cfg.Buffers.Source0))
	reg.Write(e.Base+offSource1Addr, makeAddr(cfg.Buffers.Source1))
	reg.Write(e.Base+offJPEGAddr, uint32(cfg.Buffers.JPEG))
	reg.Write(e.Base+offBCDAddr, makeAddr(cfg.Buffers.BCD))
	reg.Write(e.Base+offCompressAddr, makeAddr(cfg.Buffers.Compress))
	reg.Write(e.Base+offCBAddr, makeAddr(cfg.Buffers.Cursor))
	if cfg.Buffers.Size != 0 {
		reg.Write(e.Base+offStreamBufSize, cfg.Buffers.Size)
	}
	seq := uint32(0)
	if cfg.CompressFormat == CompressJPEG {
		seq |= seqJPEGMode | seqTrigJPEG
	}
	if cfg.YUV420 {
		seq |= seqYUV420
	}
	reg.Write(e.Base+offSeqCtrl, reg.Read(e.Base+offSeqCtrl)|seq)
}

// EnableInterrupts enables video engine interrupts.
func (e *Engine) EnableInterrupts(mask uint32) {
	if mask == 0 {
		mask = intAll
	}
	reg.Write(e.Base+offInterruptCtrl, mask)
}

// DisableInterrupts disables all video engine interrupts.
func (e *Engine) DisableInterrupts() {
	reg.Write(e.Base+offInterruptCtrl, 0)
}

// AckInterrupts returns and clears pending interrupt status.
func (e *Engine) AckInterrupts() uint32 {
	st := reg.Read(e.Base + offInterruptStatus)
	reg.Write(e.Base+offInterruptStatus, st)
	return st
}

// TriggerModeDetect starts mode detection.
func (e *Engine) TriggerModeDetect() {
	reg.SetBits32(uintptr(e.Base+offSeqCtrl), seqTrigModeDetect)
}

// DetectMode starts mode detection and waits for completion.
func (e *Engine) DetectMode(timeout time.Duration) (Geometry, error) {
	e.AckInterrupts()
	e.TriggerModeDetect()
	if err := e.WaitInterrupt(intModeDetect, timeout); err != nil {
		return Geometry{}, err
	}
	return e.Mode(), nil
}

// Mode returns the latest detected input geometry.
func (e *Engine) Mode() Geometry {
	lr := reg.Read(e.Base + offSourceLREdgeDetect)
	tb := reg.Read(e.Base + offSourceTBEdgeDetect)
	md := reg.Read(e.Base + offModeDetectStatus)
	left := int(lr & 0x0fff)
	right := int((lr >> 16) & 0x0fff)
	top := int(tb & 0x1fff)
	bottom := int((tb >> 16) & 0x1fff)
	return Geometry{Width: right - left + 1, Height: bottom - top + 1, HPeriod: int(md & 0x0fff), Interlace: lr&(1<<31) != 0, NoSignal: lr&(1<<12|1<<13|1<<14|1<<15) != 0}
}

// TriggerCapture starts a frame capture.
func (e *Engine) TriggerCapture() {
	reg.SetBits32(uintptr(e.Base+offSeqCtrl), seqTrigCapture)
}

// TriggerCompression starts compression of the captured frame.
func (e *Engine) TriggerCompression() {
	reg.SetBits32(uintptr(e.Base+offSeqCtrl), seqTrigCompress)
}

// TriggerFrame starts capture and compression for one frame.
func (e *Engine) TriggerFrame() {
	reg.SetBits32(uintptr(e.Base+offSeqCtrl), seqTrigCapture|seqTrigCompress)
}

// CaptureFrame captures and compresses one frame, returning compressed bytes.
func (e *Engine) CaptureFrame(dst []byte, timeout time.Duration) ([]byte, error) {
	e.AckInterrupts()
	e.TriggerFrame()
	if err := e.WaitInterrupt(intCaptureComplete|intCompressDone|intFrameComplete, timeout); err != nil {
		return nil, err
	}
	if err := e.WaitIdle(timeout); err != nil {
		return nil, err
	}
	n := int(e.CompressedSize())
	if n < 0 || n > len(dst) {
		n = len(dst)
	}
	return dst[:n], nil
}

// WaitIdle waits until capture and compression engines are idle.
func (e *Engine) WaitIdle(timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		if reg.Read(e.Base+offSeqCtrl)&(seqCaptureBusy|seqCompressBusy) == 0 {
			return nil
		}
	}
	return ErrTimeout
}

// WaitInterrupt waits for one or more interrupt status bits.
func (e *Engine) WaitInterrupt(mask uint32, timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		st := e.AckInterrupts()
		if st&intErrorMask != 0 {
			return errors.New("videoengine: hardware error interrupt")
		}
		if st&mask != 0 {
			return nil
		}
	}
	return ErrTimeout
}

// CompressedSize returns the last compressed frame size.
func (e *Engine) CompressedSize() uint32 {
	return reg.Read(e.Base + offCompressSizeReadback)
}

func window(width, height int) uint32 {
	return uint32(width&0x1fff) | uint32(height&0x1fff)<<16
}

func makeAddr(addr uint64) uint32 {
	return uint32(addr) | uint32(addr>>32)
}

func addr(b []byte) uint64 {
	if len(b) == 0 {
		return 0
	}
	return uint64(uintptr(unsafe.Pointer(&b[0])))
}
