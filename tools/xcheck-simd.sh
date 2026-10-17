#!/usr/bin/env bash
# Runs the box3d-go suite in the opt-in SIMD build (GOEXPERIMENT=simd, -tags box3d_simd) on an
# xcheck host. bash hosts run this file, Windows runs xcheck-simd.ps1:
#
#   xcheck run win,linux --script libs/box3d-go/tools/xcheck-simd --grep '^(--- FAIL|FAIL|ok|panic:)'
#
# On amd64 with AVX2 the suite runs at width 8; xcheck-simd-w4 runs it again at forced width 4
# (BOX3D_TEST_FORCE_WIDTH4=1, see box3d_suite_test.go).
#
# BOX3D_EXTRA_TAGS (comma separated, unset or empty for none) is appended to the build tags, e.g.
# box3d_noassert, or box3d_no8 to leave out the width 8 backend (upstream's BOX3D_DISABLE_AVX2).
# xcheck has no per-run environment, so set it in the host's [hosts.<h>.env]
# (tools/xcheck/hosts.toml or local.toml) or locally
# (BOX3D_EXTRA_TAGS=box3d_noassert bash libs/box3d-go/tools/xcheck-simd.sh).
set -euo pipefail
cd "$(dirname "$0")/.."
tags="box3d_simd"
extra="${BOX3D_EXTRA_TAGS-}"
if [ -n "$extra" ]; then
	tags="$tags,$extra"
fi
export GOEXPERIMENT=simd
echo "xcheck-simd: go test -tags $tags ./... ($(go env GOOS)/$(go env GOARCH), BOX3D_TEST_FORCE_WIDTH4=${BOX3D_TEST_FORCE_WIDTH4-})"
go test -count=1 -tags "$tags" ./...
