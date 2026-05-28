// Package ast2700net provides AST2700 Ethernet board bring-up helpers.
package ast2700net

import "github.com/kyanitecomputer/aspeed-go/reg"

const (
	SCUIOBase = 0x14c02000

	offProtect   = 0x000
	offReset1    = 0x200
	offClockStop = 0x240
	offClockClr  = 0x244
	offPinMux18  = 0x444
	offPinMux19  = 0x448
	offDrive5    = 0x4d4

	// RGMII MAC interface delay registers (per-speed). Each packs MAC0 and MAC1
	// TX/RX delays: TX0[5:0], TX1[11:6], RX0[17:12], RX1[23:18].
	offMacDelay     = 0x390 // 1000M
	offMac100mDelay = 0x394
	offMac10mDelay  = 0x398

	// RGMII delay values, dumped verbatim from a running vendor system on this
	// board (chip rev1, AN8801R rgmii-id). Field layout: TX0[5:0], TX1[11:6],
	// RX0[17:12], RX1[23:18]; decodes to MAC0 TX=25 RX=22. This is the best
	// known TX-delay point: short frames (60B ARP) transmit intact. Setting MAC
	// TX delay to 0 pushed timing fully out of the window (even ARP failed), so
	// the MAC-side TX delay is required even with the PHY in rgmii-id mode.
	macDelay1000 = 0x005d6659 // MAC0 TX=25, RX=22
	macDelay100  = 0x00410410
	macDelay10   = 0x00410410

	protectKey = 0x1688a8a8
)

// EnableMAC0RGMII enables MAC0, MDIO0, and RGMII0 pins on the AST2700 IO die.
//
// The MAC/RGMII source-clock dividers (SCU1 clk_sel1) are programmed by the
// BootMCU before it locks the clk_sel1 fields, so they cannot (and need not)
// be set here.
func EnableMAC0RGMII() {
	unlock()
	deassertReset(1<<2 | 1<<5)
	reg.Write(SCUIOBase+offClockClr, 1<<8)
	setPinMux()
	setMacDelay()
	lock()
}

// setMacDelay programs the RGMII MAC interface delays. The vendor u-boot
// calibrates these at boot; our boot path does not, so we apply the dumped
// vendor-calibrated values to fix MAC->PHY TX timing.
func setMacDelay() {
	reg.Write(SCUIOBase+offMacDelay, macDelay1000)
	reg.Write(SCUIOBase+offMac100mDelay, macDelay100)
	reg.Write(SCUIOBase+offMac10mDelay, macDelay10)
}

func unlock() {
	reg.Write(SCUIOBase+offProtect, protectKey)
}

func lock() {
	reg.Write(SCUIOBase+offProtect, 1)
}

func deassertReset(mask uint32) {
	reg.Write(SCUIOBase+offReset1+4, mask)
}

func setPinMux() {
	reg.MaskWrite32(uintptr(SCUIOBase+offPinMux18), 0x77777777, 0x11111111)
	reg.MaskWrite32(uintptr(SCUIOBase+offPinMux19), 0x00777777, 0x00111111)
	reg.MaskWrite32(uintptr(SCUIOBase+offDrive5), 0x00ffffff, 0x00555555)
}
