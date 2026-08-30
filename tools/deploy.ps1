# WaroTrans deploy — Windows PowerShell
# Deterministic path uses rsync. scp fallback is explicit because it can leave stale files.
param(
    [string]$Pi = "waro@warotrans.local",
    [string]$Run = "",
    [switch]$NoBuild,
    [switch]$DryRun,
    [switch]$NoDelete,
    [switch]$AllowScpFallback
)

$ErrorActionPreference = "Stop"

function Assert-Native([string]$Step) {
    if ($LASTEXITCODE -ne 0) {
        throw "$Step failed with exit code $LASTEXITCODE"
    }
}

if (-not (Test-Path "ros2_ws/src")) {
    throw "Run from repo root: ros2_ws/src not found."
}

Write-Host "==> Target: $Pi" -ForegroundColor Cyan
Write-Host "==> Scope : ros2_ws/src ONLY" -ForegroundColor Cyan

$rsync = Get-Command rsync -ErrorAction SilentlyContinue
if ($DryRun -and -not $rsync) {
    throw "DryRun requires rsync. scp cannot provide a deterministic dry-run."
}

if (-not $DryRun) {
    Write-Host "==> Backup remote source (best effort)" -ForegroundColor Cyan
    ssh $Pi 'mkdir -p ~/warotrans_backups; if [ -d ~/ros2_ws/src ]; then ts=$(date +%Y%m%d_%H%M%S); tar -czf ~/warotrans_backups/ros2_src_${ts}.tgz -C ~/ros2_ws src; ls -1t ~/warotrans_backups/ros2_src_*.tgz 2>/dev/null | tail -n +6 | xargs -r rm -f; fi; mkdir -p ~/ros2_ws/src ~/warotrans_tools'
    Assert-Native "Remote backup/preparation"
}

if ($rsync) {
    $rsyncArgs = @("-az", "--itemize-changes", "--exclude", "__pycache__", "--exclude", "*.pyc")
    if (-not $NoDelete) { $rsyncArgs += "--delete-after" }
    if ($DryRun) { $rsyncArgs += "--dry-run" }
    $rsyncArgs += @("ros2_ws/src/", "${Pi}:~/ros2_ws/src/")
    & rsync @rsyncArgs
    Assert-Native "rsync"
} else {
    if (-not $AllowScpFallback) {
        throw "rsync not found. Install/use rsync, or rerun with -AllowScpFallback. scp fallback can leave stale remote source files."
    }
    Write-Warning "Using scp fallback: deleted local files may remain on Pi and create ghost source."
    scp -r ros2_ws/src/* "${Pi}:~/ros2_ws/src/"
    Assert-Native "scp ROS source"
}

if ($DryRun) {
    Write-Host "==> Dry-run complete. Nothing changed." -ForegroundColor Green
    exit 0
}

scp tools/doctor.sh tools/check.sh tools/rollback_source.sh "${Pi}:~/warotrans_tools/"
Assert-Native "scp operational tools"
ssh $Pi 'chmod +x ~/warotrans_tools/doctor.sh ~/warotrans_tools/check.sh ~/warotrans_tools/rollback_source.sh'
Assert-Native "chmod operational tools"

if (-not $NoBuild) {
    Write-Host "==> Build on Pi" -ForegroundColor Cyan
    ssh $Pi 'set -e; source /opt/ros/jazzy/setup.bash; cd ~/ros2_ws; colcon build --symlink-install'
    if ($LASTEXITCODE -ne 0) {
        Write-Host "[FAIL] Build failed. Source backup remains in ~/warotrans_backups/." -ForegroundColor Red
        Write-Host "       Roll back with: .\tools\rollback.ps1" -ForegroundColor Yellow
        exit $LASTEXITCODE
    }
}

if ($Run -ne "") {
    Write-Host "==> Run $Run.launch.py" -ForegroundColor Cyan
    ssh -t $Pi "source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch warotrans_bringup $Run.launch.py"
    Assert-Native "ros2 launch"
}

Write-Host "==> Deploy complete" -ForegroundColor Green
