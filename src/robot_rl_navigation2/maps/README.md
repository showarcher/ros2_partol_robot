# Navigation maps

| Preset | Map | Matching Gazebo world | Localization |
| --- | --- | --- | --- |
| `warehouse` (default) | `warehouse_map.yaml` + `.pgm` | `world/warehouse/small_warehouse.world` | AMCL |
| `room` | `room.yaml` + `.pgm` | `world/custom_room.world` | World-aligned ground truth |

Warehouse: 318 × 439 pixels, 0.05 m/pixel, origin `[-7.56, -10.8, 0]`.
The YAML origin is the lower-left grid corner in map coordinates, not a robot spawn pose.
The world and map were imported together; preserve this pairing.
Source and license: [asset provenance](../../../docs/third_party/README.md).

Select a pair with `ros2 launch robot_rl_navigation2 navigation2.launch.py scene:=warehouse`
or `scene:=room`. Explicit `world:=... map:=... localization:=amcl` overrides support other maps.

PAL worlds do not include occupancy maps. Build one per layout using `slam:=true`,
save its YAML and PGM here, rebuild this package, and select the saved YAML with `map`.
See the [workspace guide](../../../README.md) for commands.
