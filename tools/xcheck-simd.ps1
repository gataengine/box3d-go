# Runs the box3d-go suite in the opt-in SIMD build (GOEXPERIMENT=simd, -tags box3d_simd) on an
# xcheck host. Windows runs this file, bash hosts run xcheck-simd.sh (see there for usage and
# BOX3D_EXTRA_TAGS, which is appended to the build tags; unset means box3d_no8).
$ErrorActionPreference = 'Stop'
Set-Location (Join-Path $PSScriptRoot '..')
$tags = 'box3d_simd'
$extra = $env:BOX3D_EXTRA_TAGS
if ($null -eq $extra) { $extra = 'box3d_no8' }
if ($extra -ne '') { $tags = "$tags,$extra" }
$env:GOEXPERIMENT = 'simd'
Write-Output "xcheck-simd: go test -tags $tags ./... ($(go env GOOS)/$(go env GOARCH))"
go test -count=1 -tags $tags ./...
exit $LASTEXITCODE
