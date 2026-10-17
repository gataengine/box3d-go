# Runs the box3d-go suite in the opt-in SIMD build (GOEXPERIMENT=simd, -tags box3d_simd) on an
# xcheck host. Windows runs this file, bash hosts run xcheck-simd.sh (see there for usage,
# BOX3D_TEST_FORCE_WIDTH4 and BOX3D_EXTRA_TAGS, which is appended to the build tags; unset or
# empty means none).
$ErrorActionPreference = 'Stop'
Set-Location (Join-Path $PSScriptRoot '..')
$tags = 'box3d_simd'
$extra = $env:BOX3D_EXTRA_TAGS
if ($extra) { $tags = "$tags,$extra" }
$env:GOEXPERIMENT = 'simd'
Write-Output "xcheck-simd: go test -tags $tags ./... ($(go env GOOS)/$(go env GOARCH), BOX3D_TEST_FORCE_WIDTH4=$env:BOX3D_TEST_FORCE_WIDTH4)"
go test -count=1 -tags $tags ./...
exit $LASTEXITCODE
