// Package flash defines common flash driver interfaces.
package flash

// Geometry describes common flash geometry.
type Geometry struct {
	TotalSize      int64
	EraseBlockSize int
	PageSize       int
	EraseValue     byte
}

// Device is the base interface for a flash device.
type Device interface {
	Geometry() Geometry
}

// NOR provides byte-addressed NOR flash operations.
type NOR interface {
	Device
	ReadAt(dst []byte, addr int64) (int, error)
	WriteAt(src []byte, addr int64) (int, error)
	EraseBlock(addr int64) error
}
