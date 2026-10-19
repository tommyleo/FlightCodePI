param([ValidateSet('all', 'pico2', 'pico2_w')][string]$Board = 'all')
$ErrorActionPreference = 'Stop'
$firmwareRoot = $PSScriptRoot
$picoTools = Join-Path $env:USERPROFILE '.pico-sdk'
$cmakeExe = Join-Path $picoTools 'cmake/v4.3.4/bin/cmake.exe'
$ninjaExe = Join-Path $picoTools 'ninja/v1.13.2/ninja.exe'
$toolchain = Join-Path $picoTools 'toolchain/15_2_Rel1'
$sdk = Join-Path $picoTools 'sdk/2.3.0'
foreach ($required in @($cmakeExe, $ninjaExe, (Join-Path $toolchain 'bin/arm-none-eabi-gcc.exe'), (Join-Path $sdk 'pico_sdk_init.cmake'))) {
    if (-not (Test-Path -LiteralPath $required)) { throw "Missing Pico build dependency: $required" }
}
$boards = if ($Board -eq 'all') { @('pico2', 'pico2_w') } else { @($Board) }
foreach ($targetBoard in $boards) {
    $buildName = if ($targetBoard -eq 'pico2') { 'build-release' } else { 'build-pico2-w-release' }
    $buildPath = Join-Path $firmwareRoot $buildName
    & $cmakeExe -S $firmwareRoot -B $buildPath -G Ninja "-DPICO_BOARD=$targetBoard" '-DCMAKE_BUILD_TYPE=Release' "-DPICO_SDK_PATH=$sdk" "-DPICO_TOOLCHAIN_PATH=$toolchain" "-DCMAKE_MAKE_PROGRAM=$ninjaExe"
    if ($LASTEXITCODE -ne 0) { throw "Configuration failed for $targetBoard" }
    & $cmakeExe --build $buildPath --parallel 4
    if ($LASTEXITCODE -ne 0) { throw "Build failed for $targetBoard" }
    # Export even when Ninja reports no work; output/firmware can be recreated.
    & $cmakeExe "-DFIRMWARE_SOURCE_DIR=$firmwareRoot" "-DFIRMWARE_BINARY_DIR=$buildPath" "-DFIRMWARE_BOARD=$targetBoard" -P (Join-Path $firmwareRoot 'tools/export-firmware.cmake')
    if ($LASTEXITCODE -ne 0) { throw "Firmware export failed for $targetBoard" }
}
Write-Host "Firmware ready: $(Join-Path $firmwareRoot 'output/firmware')"
