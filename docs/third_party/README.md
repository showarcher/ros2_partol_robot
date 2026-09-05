# Warehouse asset provenance

Imported on 2026-09-05 from:
https://github.com/Sara-Esm/ros2-autonomous-warehouse-navigation

Pinned upstream commit: `7abed69d7b110088f1b5240642bd7195a584d129`.

| Upstream path | Local path |
| --- | --- |
| `src/warehouse_gazebo/worlds/small_warehouse.world` | `src/robot_rl_description/world/warehouse/small_warehouse.world` |
| `src/warehouse_gazebo/models/` | `src/robot_rl_description/models/warehouse/` |
| `src/warehouse_bringup/maps/warehouse_map.{yaml,pgm}` | `src/robot_rl_navigation2/maps/warehouse_map.{yaml,pgm}` |

The world, models and navigation map are copied without geometry/map changes.
The ROS launch integration is local: FishBot is retained, model paths are resolved
from installed package shares, and warehouse spawn height is 0.08 m to clear the
floor mesh (top approximately 0.0342 m in world coordinates).

The warehouse models originate from AWS RoboMaker Small Warehouse World, included
in the upstream repository under `src/aws-robomaker-small-warehouse-world`.
Retained license texts:

- [Warehouse navigation / Sara Esmaeili](warehouse-navigation-LICENSE)
- [AWS warehouse assets / Amazon](aws-warehouse-LICENSE)

Station coordinates in `warehouse_patrol.yaml` use the upstream four station
approaches, with return to the FishBot spawn area. Marker visuals are included;
ArUco detection and the upstream TurtleBot3 mission implementation are not imported.
