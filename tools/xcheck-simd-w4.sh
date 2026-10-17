#!/usr/bin/env bash
# xcheck-simd.sh with BOX3D_TEST_FORCE_WIDTH4=1: the SIMD build's suite at forced width 4, which
# covers the width 4 kernels on amd64 hosts with AVX2. Windows runs xcheck-simd-w4.ps1:
#
#   xcheck run win,linux --script libs/box3d-go/tools/xcheck-simd-w4 --grep '^(--- FAIL|FAIL|ok|panic:)'
set -euo pipefail
export BOX3D_TEST_FORCE_WIDTH4=1
exec bash "$(dirname "$0")/xcheck-simd.sh"
