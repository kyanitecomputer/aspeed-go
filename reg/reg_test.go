package reg_test

import (
	"testing"
	"unsafe"

	"src.kyanite.computer/aspeed-go/reg"
)

// scratch creates a local uint32 and returns its address as a uintptr,
// simulating an MMIO register for testing purposes.
func scratch32() (uintptr, *uint32) {
	var v uint32
	return uintptr(unsafe.Pointer(&v)), &v
}

func TestRead32Write32(t *testing.T) {
	addr, p := scratch32()
	reg.Write32(addr, 0xDEADBEEF)
	if got := reg.Read32(addr); got != 0xDEADBEEF {
		t.Fatalf("Read32 = 0x%08X, want 0xDEADBEEF", got)
	}
	_ = p
}

func TestSetBits32(t *testing.T) {
	addr, _ := scratch32()
	reg.Write32(addr, 0x00000000)
	reg.SetBits32(addr, 0x00000005)
	if got := reg.Read32(addr); got != 0x00000005 {
		t.Fatalf("SetBits32 = 0x%08X, want 0x00000005", got)
	}
	// Setting already-set bits is idempotent.
	reg.SetBits32(addr, 0x00000005)
	if got := reg.Read32(addr); got != 0x00000005 {
		t.Fatalf("SetBits32 idempotent = 0x%08X, want 0x00000005", got)
	}
}

func TestClearBits32(t *testing.T) {
	addr, _ := scratch32()
	reg.Write32(addr, 0xFFFFFFFF)
	reg.ClearBits32(addr, 0x000000FF)
	if got := reg.Read32(addr); got != 0xFFFFFF00 {
		t.Fatalf("ClearBits32 = 0x%08X, want 0xFFFFFF00", got)
	}
}

func TestMaskWrite32(t *testing.T) {
	addr, _ := scratch32()
	reg.Write32(addr, 0xAABBCCDD)
	// Write 0x11 into the low byte, leave upper 3 bytes unchanged.
	reg.MaskWrite32(addr, 0x000000FF, 0x00000011)
	if got := reg.Read32(addr); got != 0xAABBCC11 {
		t.Fatalf("MaskWrite32 = 0x%08X, want 0xAABBCC11", got)
	}
}

func TestRead16Write16(t *testing.T) {
	var v uint16
	addr := uintptr(unsafe.Pointer(&v))
	reg.Write16(addr, 0x1234)
	if got := reg.Read16(addr); got != 0x1234 {
		t.Fatalf("Read16 = 0x%04X, want 0x1234", got)
	}
}

func TestRead8Write8(t *testing.T) {
	var v uint8
	addr := uintptr(unsafe.Pointer(&v))
	reg.Write8(addr, 0xAB)
	if got := reg.Read8(addr); got != 0xAB {
		t.Fatalf("Read8 = 0x%02X, want 0xAB", got)
	}
}
