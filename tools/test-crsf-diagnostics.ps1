param([string]$Compiler = "gcc")
$ErrorActionPreference = "Stop"
$crsfRoot = Split-Path -Parent $PSScriptRoot
Push-Location $crsfRoot
try {
    New-Item -ItemType Directory -Path build -Force | Out-Null
    $crsfSource = Get-Content src/drivers/receiver/sbus_receiver.c -Raw
    foreach ($crsfPart in @(
        @('static uint8_t crsf_crc8(', 'static uint16_t sbus_raw_to_us(', 'crsf_crc_under_test.inc'),
        @('static void accept_byte(', 'static void accept_serial_word(', 'crsf_accept_under_test.inc')
    )) {
        $crsfStart = $crsfSource.IndexOf($crsfPart[0])
        $crsfEnd = $crsfSource.IndexOf($crsfPart[1], $crsfStart)
        if ($crsfStart -lt 0 -or $crsfEnd -le $crsfStart) { throw "Missing CRSF parser" }
        [IO.File]::WriteAllText((Join-Path $crsfRoot ('build/' + $crsfPart[2])), $crsfSource.Substring($crsfStart, $crsfEnd - $crsfStart))
    }
    & $Compiler -std=c11 -Wall -Wextra -Werror -Ibuild -Isrc/drivers/receiver tests/crsf_diagnostics_test.c -o build/crsf-diagnostics-test.exe
    if ($LASTEXITCODE -ne 0) { throw "CRSF diagnostics test compilation failed" }
    & ./build/crsf-diagnostics-test.exe
    if ($LASTEXITCODE -ne 0) { throw "CRSF diagnostics test failed" }
} finally { Pop-Location }
