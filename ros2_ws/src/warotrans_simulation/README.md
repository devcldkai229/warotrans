# warotrans_simulation

Robot WaroTrans ảo trong **Gazebo Harmonic** (ADR 0006). Chạy cùng Nav2, AMCL và gateway MQTT như robot thật,
nên backend điều khiển nó qua đúng MQTT contract.

```bash
ros2 launch warotrans_simulation sim_bringup.launch.py              # Gazebo + AMCL + Nav2 + gateway RBT-SIM-01
ros2 launch warotrans_simulation sim_bringup.launch.py rviz:=true gui:=false
ros2 launch warotrans_simulation simulation.launch.py               # chỉ Gazebo + robot + bridge
python3 scripts/generate_warehouse.py                               # sinh lại world/map/endpoints từ layout
```

- Layout kho (nguồn duy nhất): `config/warehouse_layout.yaml`
- File sinh ra (không sửa tay): `worlds/warehouse.sdf`, `maps/sim_warehouse.*`, `config/sim_endpoints.json`

Giải thích đầy đủ: `docs/simulation/` (`00-moi-truong-wsl2.md`, `01-robot-ao-gazebo.md`).
