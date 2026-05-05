module github.com/kyanitecomputer/aspeed-go

go 1.24

// Zero external dependencies — the reg package uses only unsafe and the
// Go standard library. HAL drivers use only the reg package.
