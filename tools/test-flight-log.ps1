param([string]$Compiler = 'C:\msys64\ucrt64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
if (Test-Path -LiteralPath $Compiler) {
    $env:PATH = (Split-Path -Parent $Compiler) + ';' + $env:PATH
}
$testOutput = Join-Path $root 'build-release'
New-Item -ItemType Directory -Path $testOutput -Force | Out-Null
foreach ($test in @('flight_log_test', 'controller_log_signals_test', 'log_metadata_test', 'gyro_rate_limit_test')) {
    $exe = Join-Path $testOutput "$test.exe"
    & $Compiler -std=c11 -Wall -Wextra -Werror "-I$root/tests/log_stubs" "-I$root/src/drivers/imu" "-I$root/src/control" "-I$root/src/storage" "-I$root/include" "$root/tests/$test.c" -lm -o $exe
    if ($LASTEXITCODE -ne 0) { throw "$test compilation failed" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "$test failed" }
}
