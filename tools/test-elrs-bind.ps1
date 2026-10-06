param([string]$Compiler = "gcc")
$ErrorActionPreference = "Stop"
$bindRoot = Split-Path -Parent $PSScriptRoot
Push-Location $bindRoot
try {
    New-Item -ItemType Directory -Path build -Force | Out-Null
    $bindSource = (Get-Content src/protocol/config_protocol.c -Raw).Replace("`r`n", "`n")
    $bindCommand = $bindSource.IndexOf('    if (strcmp(command, "BIND_RECEIVER") == 0)')
    if ($bindCommand -lt 0) { throw "Missing binding command" }
    $bindStart = $bindSource.LastIndexOf('    if (flight_control_is_armed()) {', $bindCommand)
    if ($bindStart -lt 0) { $bindStart = $bindSource.LastIndexOf('    if (armed) {', $bindCommand) }
    $bindSuffix = "`n        return;`n    }"
    $bindEnd = $bindSource.IndexOf($bindSuffix, $bindCommand) + $bindSuffix.Length
    if ($bindStart -lt 0 -or $bindEnd -le $bindCommand) { throw "Missing binding guards" }
    [IO.File]::WriteAllText((Join-Path $bindRoot 'build/elrs_bind_command_under_test.inc'), $bindSource.Substring($bindStart, $bindEnd - $bindStart))
    & $Compiler -std=c11 -Wall -Wextra -Wno-unused-function -Ibuild -Isrc/drivers/receiver tests/elrs_bind_test.c -o build/elrs-bind-test.exe
    if ($LASTEXITCODE -ne 0) { throw "ELRS bind test compilation failed" }
    & ./build/elrs-bind-test.exe
    if ($LASTEXITCODE -ne 0) { throw "ELRS bind test failed" }
} finally { Pop-Location }
