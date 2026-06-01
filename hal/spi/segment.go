package spi

// SoC selects the FMC/SPI controller generation. The chip-enable address
// decoding ("segment") register at offset 0x30+4*cs is encoded differently on
// each generation: the unit size, which address bits participate in decoding,
// and whether the end bound is inclusive or exclusive all vary.
//
// Both generations store the start bound in register bits [15:0] and the end
// bound in bits [31:16]; each field holds address bits A[31:16] of a
// window-relative offset. The differences are purely in interpretation.
type SoC int

const (
	// SoCUnknown is the zero value and is rejected by the segment helpers.
	SoCUnknown SoC = iota

	// AST2600 covers AST2600 / AST1030 / AST1060 (fmc_v1). Segments decode
	// only A[27:20] (1 MiB units) and the end bound is inclusive.
	AST2600

	// AST2700 covers the AST2700 system SPI controllers (spi_ast2700_v1).
	// Segments decode all of A[31:16] (64 KiB units) and the end bound is
	// exclusive.
	AST2700
)

// segParams captures the per-SoC segment encoding rules.
type segParams struct {
	unit     int64  // decode granularity in bytes
	decodeHi uint32 // window-relative byte-address mask of decoded bits
}

func (s SoC) seg() (segParams, bool) {
	switch s {
	case AST2600:
		// 1 MiB units, only A[27:20] decode → 256 MiB window.
		return segParams{unit: 0x10_0000, decodeHi: 0x0ff0_0000}, true
	case AST2700:
		// 64 KiB units, all A[31:16] decode.
		return segParams{unit: 0x1_0000, decodeHi: 0xffff_0000}, true
	default:
		return segParams{}, false
	}
}

// segFields splits a raw segment register value into its start and end fields.
func segFields(regVal uint32) (start, end uint32) {
	return regVal & 0xffff, (regVal >> 16) & 0xffff
}

// SegmentDecode decodes a chip-enable address-decoding register value into a
// window-relative start offset and the decoded window size in bytes.
//
// A disabled segment (start field == end field, e.g. a zero register) decodes
// to size 0. ok is false for an unsupported SoC.
func SegmentDecode(soc SoC, regVal uint32) (startOff int64, size int64, ok bool) {
	p, ok := soc.seg()
	if !ok {
		return 0, 0, false
	}
	sf, ef := segFields(regVal)

	// Window-relative byte address of the start bound, keeping only the
	// bits this generation actually decodes.
	startOff = int64((sf << 16) & p.decodeHi)

	if sf == ef {
		// Decoding disabled.
		return startOff, 0, true
	}

	// End bound, keeping only the decoded bits. On AST2600 this is the base
	// of the last decoded unit (inclusive); on AST2700 it is the exclusive
	// upper bound. Normalise both to an exclusive end.
	endBound := int64((ef << 16) & p.decodeHi)
	switch soc {
	case AST2600:
		// Inclusive: the range extends to the end of the last unit.
		endOff := endBound + p.unit
		return startOff, endOff - startOff, true
	default: // AST2700
		// Exclusive already.
		return startOff, endBound - startOff, true
	}
}

// SegmentEncode builds a chip-enable address-decoding register value for a
// window-relative start offset and size in bytes. start and size must be
// aligned to the SoC's unit size; a zero size produces a disabled segment.
// ok is false for an unsupported SoC or misaligned/negative inputs.
func SegmentEncode(soc SoC, startOff, size int64) (regVal uint32, ok bool) {
	p, ok := soc.seg()
	if !ok {
		return 0, false
	}
	if startOff < 0 || size < 0 || startOff%p.unit != 0 || size%p.unit != 0 {
		return 0, false
	}

	sf := uint32(startOff>>16) & (p.decodeHi >> 16)
	if size == 0 {
		// Disabled: start field == end field.
		return sf | (sf << 16), true
	}

	endExcl := startOff + size
	var ef uint32
	switch soc {
	case AST2600:
		// Inclusive end field = base of the last decoded unit.
		ef = uint32((endExcl-1)>>16) & (p.decodeHi >> 16)
	default: // AST2700
		ef = uint32(endExcl>>16) & (p.decodeHi >> 16)
	}
	return sf | (ef << 16), true
}
