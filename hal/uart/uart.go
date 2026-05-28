// Package uart provides helpers for ASPEED 16550-compatible UARTs.
package uart

import "github.com/kyanitecomputer/aspeed-go/reg"

const (
	rbrThr = 0x00
	ier    = 0x04
	iirFcr = 0x08
	lcr    = 0x0c
	lsr    = 0x14

	lcrWordLen8 = 0x03
	lcrDLAB     = 1 << 7

	fcrEnable  = 1 << 0
	fcrRxReset = 1 << 1
	fcrTxReset = 1 << 2

	lsrTHRE = 1 << 5
	lsrTEMT = 1 << 6
)

// UART represents an ASPEED 16550-compatible UART register block.
type UART struct {
	Base uint32
}

// Init115200 configures the UART for 115200 8N1 using divisor 1.
func (u UART) Init115200() {
	reg.Write(u.Base+lcr, lcrDLAB)
	reg.Write(u.Base+rbrThr, 1)
	reg.Write(u.Base+ier, 0)
	reg.Write(u.Base+lcr, lcrWordLen8)
	reg.Write(u.Base+iirFcr, fcrEnable|fcrRxReset|fcrTxReset)
}

// Tx writes one byte after waiting for TX holding register empty.
func (u UART) Tx(b byte) {
	for reg.Read(u.Base+lsr)&lsrTHRE == 0 {
	}
	reg.Write(u.Base+rbrThr, uint32(b))
}

// Write writes all bytes and waits for the TX shift register to drain.
func (u UART) Write(p []byte) (int, error) {
	for _, b := range p {
		u.Tx(b)
	}
	for reg.Read(u.Base+lsr)&lsrTEMT == 0 {
	}
	return len(p), nil
}
