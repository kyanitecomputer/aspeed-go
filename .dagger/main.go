// Dagger CI module for aspeed-go.
//
// Usage (from the aspeed-go repo root):
//
//	dagger call ci      # build + test + vet
//	dagger call test    # go test ./...
//	dagger call build   # go build ./...
package main

import (
	"context"
	"strings"

	"dagger/aspeed-go/internal/dagger"
)

const goVersion = "1.24"

type AspeedGo struct{}

func (m *AspeedGo) goContainer(src *dagger.Directory) *dagger.Container {
	goCache := dag.CacheVolume("go-mod-cache")
	goBuild := dag.CacheVolume("go-build-cache")
	return dag.Container().
		From("golang:" + goVersion + "-bookworm").
		WithMountedCache("/go/pkg/mod", goCache).
		WithMountedCache("/root/.cache/go-build", goBuild).
		WithDirectory("/src", src).
		WithWorkdir("/src")
}

// Build runs go build ./... to verify all packages compile.
func (m *AspeedGo) Build(
	ctx context.Context,
	// +defaultPath="."
	src *dagger.Directory,
) error {
	_, err := m.goContainer(src).
		WithExec([]string{"go", "build", "./..."}).
		Sync(ctx)
	return err
}

// Test runs go test ./... including the reg package unit tests.
func (m *AspeedGo) Test(
	ctx context.Context,
	// +defaultPath="."
	src *dagger.Directory,
) error {
	_, err := m.goContainer(src).
		WithExec([]string{"go", "test", "-v", "-count=1", "./..."}).
		Sync(ctx)
	return err
}

// Vet runs go vet ./... for static analysis.
// The unsafeptr check is disabled: reg/ intentionally casts uintptr→unsafe.Pointer
// for volatile MMIO access, which is correct for bare-metal targets.
func (m *AspeedGo) Vet(
	ctx context.Context,
	// +defaultPath="."
	src *dagger.Directory,
) error {
	_, err := m.goContainer(src).
		WithExec([]string{"go", "vet", "-unsafeptr=false", "./..."}).
		Sync(ctx)
	return err
}

// Ci runs the full pipeline: Build + Test + Vet.
func (m *AspeedGo) Ci(
	ctx context.Context,
	// +defaultPath="."
	src *dagger.Directory,
) (string, error) {
	steps := []string{}

	if err := m.Build(ctx, src); err != nil {
		return "", err
	}
	steps = append(steps, "build: ok")

	if err := m.Test(ctx, src); err != nil {
		return "", err
	}
	steps = append(steps, "test: ok")

	if err := m.Vet(ctx, src); err != nil {
		return "", err
	}
	steps = append(steps, "vet: ok")

	return strings.Join(steps, "\n"), nil
}
