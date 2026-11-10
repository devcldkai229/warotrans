# Two-robot, motion-free traffic E2E against the real backend + broker (isolated DB and topic tree).
#   powershell -ExecutionPolicy Bypass -File tools/traffic_e2e/run_traffic_e2e.ps1
# Needs: docker containers warotrans-postgres / warotrans-mosquitto / warotrans-mongodb running; no other Host on 5199.
# Writes only to the warotrans_test database and the warotrans-e2e/v1 MQTT topics. Never moves a real robot.
param([string]$SystemRepo = (Join-Path $PSScriptRoot "../../../warotrans-system"), [string]$Filter = "")
$ErrorActionPreference = "Stop"
$hostProj = Join-Path $SystemRepo "backend/src/WaroTrans.Host"
dotnet build $hostProj --nologo -v q
if ($LASTEXITCODE -ne 0) { throw "Host build failed" }
$env:WAROTRANS_SYSTEM_REPO = (Resolve-Path $SystemRepo).Path
Push-Location $PSScriptRoot
try {
    $args = @("-m", "pytest", "test_traffic_e2e.py", "-v", "-p", "no:cacheprovider")
    if ($Filter) { $args += @("-k", $Filter) }
    python @args
    exit $LASTEXITCODE
} finally { Pop-Location }
