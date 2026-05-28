# aspeed-go — Roadmap

## Phase A: Foundation (complete)

| Task | Description | Status |
|------|-------------|--------|
| A-1 | `reg` package: `Read32`/`Write32`/`Read16`/`Write16`/`Read8`/`Write8`/`Read64`/`Write64` | ✅ |
| A-2 | `reg` package: `SetBits32`/`ClearBits32`/`MaskWrite32` helpers | ✅ |
| A-3 | Unit tests for all reg functions | ✅ |
| A-4 | `go build ./...` and `go test ./...` verified | ✅ |

## Phase B: PAC population (partially unblocked)

The chiptool Go backend (`generate::go`) is implemented and wired into
`aspeed-data-gen`.  Generated Go PAC files for all five chip targets now
live in `aspeed-data/aspeed-go-pac/` and are produced by `cargo run -p
aspeed-data-gen`.

The generated module is `github.com/kyanitecomputer/aspeed-data/aspeed-go-pac`
and it imports `github.com/kyanitecomputer/aspeed-go/reg` for register
access.  For local development use a `go.work` file:

```sh
go 1.24
use .
use ../aspeed-data/aspeed-go-pac
```

The Go PAC is generated but **not yet consumed by any HAL driver in this
repo**.  The `hal/` packages remain stubs until the tasks below are done.

| Task | Description | Status |
|------|-------------|--------|
| B-0 | chiptool Go backend wired into aspeed-data-gen | ✅ |
| B-1 | AST2600 UART driver using generated `uart_v1` PAC struct | ❌ |
| B-2 | AST2600 GPIO driver using generated `gpio_v1` PAC struct | ❌ |
| B-3 | AST2600 IPC doorbell driver (CA35 side, via `ipc_v1`) | ❌ |

## Phase C: TamaGo integration

| Task | Description |
|------|-------------|
| C-1 | Verify `go build` with `GOOS=tamago GOARCH=arm` for AST2600 (Cortex-A7) |
| C-2 | Verify `go build` with `GOOS=tamago GOARCH=arm64` for AST2700 (Cortex-A35) |
| C-3 | Add SoC package stub `soc/aspeed/ast2600/` following TamaGo conventions |

## Phase D: AST2500/AST2400 ColdFire support

The ColdFire coprocessors are accessed from the ARM main CPU (CA7/ARM926/ARM1176) side over shared DRAM. The Go HAL running on the main CPU can control coprocessor peripherals via MMIO at the physical addresses defined in the chip YAML.

| Task | Description |
|------|-------------|
| D-1 | AST2500 coprocessor control driver (via `coproc_v1` PAC) |
| D-2 | AST2400 coprocessor control driver (via `coproc_v2` PAC) |

## Post-push dependency note

Once `aspeed-data` is pushed to `github.com/kyanitecomputer/aspeed-data`, the `pac/` generation workflow changes from manual copy to:

```sh
# Add aspeed-data as a Go module (future, once Go PAC is published)
go get github.com/kyanitecomputer/aspeed-data/aspeed-go-pac@<sha>
```

Until then, generated files are copied manually into `pac/`.
