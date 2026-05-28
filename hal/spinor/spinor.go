// Package spinor provides a JEDEC SPI NOR flash driver.
package spinor

import (
	"errors"
	"fmt"

	"github.com/kyanitecomputer/aspeed-go/hal/flash"
	"github.com/kyanitecomputer/aspeed-go/hal/spi"
)

const (
	cmdReadStatus  = 0x05
	cmdReadData    = 0x03
	cmdWriteEnable = 0x06
	cmdPageProgram = 0x02
	cmdSectorErase = 0x20

	statusWIP = 1 << 0

	defaultPageSize  = 256
	defaultEraseSize = 4096
)

var (
	ErrOutOfRange = errors.New("spinor: out of range")
	ErrTimeout    = errors.New("spinor: operation timeout")
)

// NOR is a byte-addressed SPI NOR flash device.
type NOR struct {
	Bus       *spi.Device
	Size      int64
	PageSize  int
	EraseSize int
}

// Init initializes default geometry from the SPI decoded window.
func (n *NOR) Init() error {
	if n.Bus == nil {
		return errors.New("spinor: nil SPI device")
	}
	if n.Size == 0 {
		n.Size = n.Bus.WindowSize
	}
	if n.PageSize == 0 {
		n.PageSize = defaultPageSize
	}
	if n.EraseSize == 0 {
		n.EraseSize = defaultEraseSize
	}
	return nil
}

// Geometry returns the NOR flash geometry.
func (n *NOR) Geometry() flash.Geometry {
	return flash.Geometry{TotalSize: n.Size, EraseBlockSize: n.EraseSize, PageSize: n.PageSize, EraseValue: 0xff}
}

// ReadAt reads from the memory-mapped flash window.
func (n *NOR) ReadAt(dst []byte, addr int64) (int, error) {
	if err := n.check(addr, len(dst)); err != nil {
		return 0, err
	}
	n.Bus.TxRx(appendAddr([]byte{cmdReadData}, addr), dst)
	return len(dst), nil
}

// WriteAt page-programs bytes into the flash.
func (n *NOR) WriteAt(src []byte, addr int64) (int, error) {
	if err := n.check(addr, len(src)); err != nil {
		return 0, err
	}
	written := 0
	for written < len(src) {
		pageOff := int((addr + int64(written)) % int64(n.PageSize))
		chunk := min(n.PageSize-pageOff, len(src)-written)
		if err := n.programPage(addr+int64(written), src[written:written+chunk]); err != nil {
			return written, err
		}
		written += chunk
	}
	return written, nil
}

// EraseBlock erases the block containing addr.
func (n *NOR) EraseBlock(addr int64) error {
	if addr < 0 || addr >= n.Size || addr%int64(n.EraseSize) != 0 {
		return ErrOutOfRange
	}
	if err := n.writeEnable(); err != nil {
		return err
	}
	n.Bus.TxRx(appendAddr([]byte{cmdSectorErase}, addr), nil)
	return n.waitReady()
}

func (n *NOR) programPage(addr int64, src []byte) error {
	if err := n.writeEnable(); err != nil {
		return err
	}
	cmd := appendAddr([]byte{cmdPageProgram}, addr)
	cmd = append(cmd, src...)
	n.Bus.TxRx(cmd, nil)
	return n.waitReady()
}

func (n *NOR) writeEnable() error {
	n.Bus.TxRx([]byte{cmdWriteEnable}, nil)
	return nil
}

func (n *NOR) waitReady() error {
	var st [1]byte
	for i := 0; i < 1000000; i++ {
		n.Bus.TxRx([]byte{cmdReadStatus}, st[:])
		if st[0]&statusWIP == 0 {
			return nil
		}
	}
	return ErrTimeout
}

func (n *NOR) check(addr int64, size int) error {
	if addr < 0 || size < 0 || addr+int64(size) > n.Size {
		return fmt.Errorf("%w: addr=%d size=%d", ErrOutOfRange, addr, size)
	}
	return nil
}

func appendAddr(cmd []byte, addr int64) []byte {
	return append(cmd, byte(addr>>16), byte(addr>>8), byte(addr))
}
