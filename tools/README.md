# WaroTrans Tools

## `check.sh`

Quality gate trước deploy.

```bash
./tools/check.sh
./tools/check.sh --strict
```

- static policy checks;
- `colcon build/test` nếu ROS workspace/toolchain có sẵn;
- PlatformIO build nếu firmware project + `pio` có sẵn.

`--strict` biến thiếu toolchain cần thiết thành failure.

## `deploy.sh`

Linux/macOS/WSL/Git Bash:

```bash
./tools/deploy.sh --dry-run
./tools/deploy.sh
./tools/deploy.sh --run mapping
```

Default:
- backup `~/ros2_ws/src` trên Pi;
- mirror **chỉ** `ros2_ws/src`;
- không xóa maps/bags/logs ngoài source;
- giữ 5 backup source gần nhất;
- copy runtime diagnostic tools;
- build trên Pi.

## `deploy.ps1`

Windows PowerShell:

```powershell
.\tools\deploy.ps1 -DryRun
.\tools\deploy.ps1
.\tools\deploy.ps1 -Run mapping
```

Khuyến nghị dùng `rsync`. `-AllowScpFallback` là opt-in vì scp không xóa stale source.

## `doctor.sh`

Chạy trên Pi sau deploy:

```bash
~/warotrans_tools/doctor.sh idle
~/warotrans_tools/doctor.sh base
~/warotrans_tools/doctor.sh mapping
~/warotrans_tools/doctor.sh navigation
```

Hoặc Windows:

```powershell
.\tools\doctor.ps1 -Mode mapping
```

Doctor thu thập evidence, không tự thay calibration.

## Rollback source

Nếu deploy mới hỏng:

```powershell
.\tools\rollback.ps1
```

hoặc trên Pi:

```bash
~/warotrans_tools/rollback_source.sh
```

Sau rollback phải build/test lại trước khi bật motion.
