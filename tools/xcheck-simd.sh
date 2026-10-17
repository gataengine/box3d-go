#!/usr/bin/env bash
# Runs the box3d-go suite in the opt-in SIMD build (GOEXPERIMENT=simd, -tags box3d_simd) on an
# xcheck host. bash hosts run this file, Windows runs xcheck-simd.ps1:
#
#   xcheck run win,linux --script libs/box3d-go/tools/xcheck-simd --grep '^(--- FAIL|FAIL|ok|panic:)'
#
# BOX3D_EXTRA_TAGS (comma separated) is appended to the build tags. xcheck has no per-run
# environment, so set it in the host's [hosts.<h>.env] (tools/xcheck/hosts.toml or local.toml)
# or locally (BOX3D_EXTRA_TAGS=box3d_noassert bash libs/box3d-go/tools/xcheck-simd.sh). When it
# is unset the default is box3d_no8: the generated width 8 kernels (*8_gen.go) do not compile
# until their lane backend (simd_x8_amd64.go) exists. Set it to the empty string to build them.
set -euo pipefail
cd "$(dirname "$0")/.."
tags="box3d_simd"
extra="${BOX3D_EXTRA_TAGS-box3d_no8}"
if [ -n "$extra" ]; then
	tags="$tags,$extra"
fi
export GOEXPERIMENT=simd
echo "xcheck-simd: go test -tags $tags ./... ($(go env GOOS)/$(go env GOARCH))"
go test -count=1 -tags "$tags" ./...
