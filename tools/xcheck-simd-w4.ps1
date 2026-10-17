# xcheck-simd.ps1 with BOX3D_TEST_FORCE_WIDTH4=1 (see xcheck-simd-w4.sh).
$ErrorActionPreference = 'Stop'
$env:BOX3D_TEST_FORCE_WIDTH4 = '1'
& (Join-Path $PSScriptRoot 'xcheck-simd.ps1')
exit $LASTEXITCODE
