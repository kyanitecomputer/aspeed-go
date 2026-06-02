// Package spinor provides a JEDEC SPI NOR flash driver.
package spinor

import (
	"errors"
	"fmt"
	"runtime"
	"sync"
	"time"

	"github.com/kyanitecomputer/aspeed-go/hal/flash"
	"github.com/kyanitecomputer/aspeed-go/hal/spi"
)

// SPI NOR opcodes. Reads use the controller's memory-mapped auto-read window;
// erase and program use dedicated 4-byte-address opcodes so the whole device
// (including addresses above 16 MiB) is reachable without EN4B mode changes.
const (
	cmdReadStatus  = 0x05
	cmdWriteEnable = 0x06
	cmdReadID      = 0x9F
	cmdPageProgram = 0x12 // page program, 4-byte address
	cmdSectorErase = 0x21 // sector erase (4 KiB), 4-byte address

	statusWIP = 1 << 0 // write in progress
	statusWEL = 1 << 1 // write enable latch

	defaultPageSize  = 256
	defaultEraseSize = 4096

	// Worst-case bounds for a Macronix-class device; polling exits as soon as
	// WIP clears, so these only cap a stuck operation.
	eraseTimeout   = 5 * time.Second
	programTimeout = 1 * time.Second
)

var (
	ErrOutOfRange  = errors.New("spinor: out of range")
	ErrTimeout     = errors.New("spinor: operation timeout")
	ErrWriteEnable = errors.New("spinor: write enable not latched")
	ErrWriteFail   = errors.New("spinor: program failed")
	ErrEraseFail   = errors.New("spinor: erase failed")
)

// NOR is a byte-addressed SPI NOR flash device.
//
// Its public operations are mutually exclusive: the underlying SPI controller
// has global per-CE state (command mode, CE assert) that a user-mode command
// mutates, so concurrent transactions would corrupt each other. The mutex lets
// several logical volumes (e.g. a JetStream store and a config store carved
// from disjoint regions of the same chip) share one NOR safely.
type NOR struct {
	Bus       *spi.Device
	Size      int64
	PageSize  int
	EraseSize int

	mu sync.Mutex
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

// ReadID returns the 3-byte JEDEC RDID response (manufacturer, memory type,
// capacity code).
func (n *NOR) ReadID() [3]byte {
	n.mu.Lock()
	defer n.mu.Unlock()
	var id [3]byte
	n.Bus.Command([]byte{cmdReadID}, id[:])
	return id
}

// Geometry returns the NOR flash geometry.
func (n *NOR) Geometry() flash.Geometry {
	return flash.Geometry{TotalSize: n.Size, EraseBlockSize: n.EraseSize, PageSize: n.PageSize, EraseValue: 0xff}
}

// ReadAt reads from the memory-mapped auto-read window.
func (n *NOR) ReadAt(dst []byte, addr int64) (int, error) {
	if err := n.check(addr, len(dst)); err != nil {
		return 0, err
	}
	n.mu.Lock()
	defer n.mu.Unlock()
	n.Bus.ReadWindow(dst, addr)
	return len(dst), nil
}

// WriteAt page-programs bytes into the flash.
func (n *NOR) WriteAt(src []byte, addr int64) (int, error) {
	if err := n.check(addr, len(src)); err != nil {
		return 0, err
	}
	n.mu.Lock()
	defer n.mu.Unlock()
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
	n.mu.Lock()
	defer n.mu.Unlock()
	if err := n.writeEnable(); err != nil {
		return err
	}
	n.Bus.Command(appendAddr([]byte{cmdSectorErase}, addr), nil)
	if err := n.waitReady(eraseTimeout); err != nil {
		return fmt.Errorf("%w at %#x: %w", ErrEraseFail, addr, err)
	}
	return nil
}

func (n *NOR) programPage(addr int64, src []byte) error {
	if err := n.writeEnable(); err != nil {
		return err
	}
	cmd := appendAddr([]byte{cmdPageProgram}, addr)
	cmd = append(cmd, src...)
	n.Bus.Command(cmd, nil)
	if err := n.waitReady(programTimeout); err != nil {
		return fmt.Errorf("%w at %#x: %w", ErrWriteFail, addr, err)
	}
	return nil
}

// writeEnable issues WREN and verifies the write-enable latch actually set.
func (n *NOR) writeEnable() error {
	n.Bus.Command([]byte{cmdWriteEnable}, nil)
	if n.status()&statusWEL == 0 {
		return ErrWriteEnable
	}
	return nil
}

func (n *NOR) status() byte {
	var st [1]byte
	n.Bus.Command([]byte{cmdReadStatus}, st[:])
	return st[0]
}

// waitReady polls the status register until WIP clears or timeout elapses,
// yielding the scheduler between polls so other goroutines can run.
func (n *NOR) waitReady(timeout time.Duration) error {
	deadline := time.Now().Add(timeout)
	for {
		if n.status()&statusWIP == 0 {
			return nil
		}
		if time.Now().After(deadline) {
			return ErrTimeout
		}
		runtime.Gosched()
	}
}

func (n *NOR) check(addr int64, size int) error {
	if addr < 0 || size < 0 || addr+int64(size) > n.Size {
		return fmt.Errorf("%w: addr=%d size=%d", ErrOutOfRange, addr, size)
	}
	return nil
}

func appendAddr(cmd []byte, addr int64) []byte {
	return append(cmd, byte(addr>>24), byte(addr>>16), byte(addr>>8), byte(addr))
}
