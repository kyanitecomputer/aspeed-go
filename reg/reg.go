// Package reg provides volatile MMIO register access primitives for bare-metal
// Go targets (TamaGo, GOOS=tamago).
//
// All functions operate directly on physical memory addresses via unsafe.Pointer.
// No system calls, no OS, no allocations. Safe to call from interrupt context.
//
// Intended use:
//
//	import "github.com/kyanitecomputer/aspeed-go/reg"
//
//	// Read the SCU protection register on AST2600
//	v := reg.Read32(0x1E6E2000)
//	// Unlock SCU
//	reg.Write32(0x1E6E2000, 0x1688A8A8)
package reg

import "unsafe"

// Read32 reads a 32-bit value from a MMIO register at addr.
// The read is volatile — the compiler will not cache or elide it.
//
// The uintptr→unsafe.Pointer conversion is intentional: addr is a physical
// hardware register address, not a Go heap pointer. go vet's unsafeptr check
// is suppressed for this package with -unsafeptr=false in CI.
//
//go:nosplit
func Read32(addr uintptr) uint32 {
	return *(*uint32)(unsafe.Pointer(addr)) //nolint:unsafeptr
}

// Write32 writes a 32-bit value to a MMIO register at addr.
// The write is volatile — the compiler will not cache or elide it.
//
//go:nosplit
func Write32(addr uintptr, val uint32) {
	*(*uint32)(unsafe.Pointer(addr)) = val
}

// Read16 reads a 16-bit value from a MMIO register at addr.
//
//go:nosplit
func Read16(addr uintptr) uint16 {
	return *(*uint16)(unsafe.Pointer(addr))
}

// Write16 writes a 16-bit value to a MMIO register at addr.
//
//go:nosplit
func Write16(addr uintptr, val uint16) {
	*(*uint16)(unsafe.Pointer(addr)) = val
}

// Read8 reads an 8-bit value from a MMIO register at addr.
//
//go:nosplit
func Read8(addr uintptr) uint8 {
	return *(*uint8)(unsafe.Pointer(addr))
}

// Write8 writes an 8-bit value to a MMIO register at addr.
//
//go:nosplit
func Write8(addr uintptr, val uint8) {
	*(*uint8)(unsafe.Pointer(addr)) = val
}

// Read64 reads a 64-bit value from a MMIO register at addr.
// Used for 64-bit registers on 64-bit targets (AST2700 CA35).
//
//go:nosplit
func Read64(addr uintptr) uint64 {
	return *(*uint64)(unsafe.Pointer(addr))
}

// Write64 writes a 64-bit value to a MMIO register at addr.
//
//go:nosplit
func Write64(addr uintptr, val uint64) {
	*(*uint64)(unsafe.Pointer(addr)) = val
}

// SetBits32 sets the given bits in a 32-bit MMIO register (read-modify-write).
//
//go:nosplit
func SetBits32(addr uintptr, mask uint32) {
	Write32(addr, Read32(addr)|mask)
}

// ClearBits32 clears the given bits in a 32-bit MMIO register (read-modify-write).
//
//go:nosplit
func ClearBits32(addr uintptr, mask uint32) {
	Write32(addr, Read32(addr)&^mask)
}

// MaskWrite32 writes val into the masked field of a 32-bit MMIO register.
// Equivalent to: reg = (reg & ~mask) | (val & mask)
//
//go:nosplit
func MaskWrite32(addr uintptr, mask, val uint32) {
	Write32(addr, (Read32(addr)&^mask)|(val&mask))
}
