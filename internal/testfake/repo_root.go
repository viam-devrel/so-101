// Package testfake holds test-only helpers shared across the module's test files.
package testfake

import (
	"os"
	"path/filepath"
	"runtime"
)

// RepoRoot returns the absolute path to the repository root (the directory containing
// go.mod), located by walking up from this source file's own location. Tests use it as
// VIAM_MODULE_ROOT so runtime-loaded assets (assets/urdf/so101.urdf, ...) resolve the same
// way viam-server would resolve them from the module's unpacked tarball root -- independent
// of the package's location in the Go module tree.
func RepoRoot() string {
	_, thisFile, _, _ := runtime.Caller(0)
	dir := filepath.Dir(thisFile)
	for {
		if _, err := os.Stat(filepath.Join(dir, "go.mod")); err == nil {
			return dir
		}
		parent := filepath.Dir(dir)
		if parent == dir {
			// Never reached in a normal checkout. It IS reached under -trimpath, where
			// runtime.Caller returns a relative path -- panic rather than hand back a
			// bogus root that would surface later as a confusing missing-asset failure.
			panic("testfake.RepoRoot: no go.mod above " + thisFile)
		}
		dir = parent
	}
}
