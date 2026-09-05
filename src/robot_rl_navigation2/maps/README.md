# Navigation maps

`room.yaml` and `room.pgm` match `robot_rl_description/world/custom_room.world`.

The migrated PAL Gazebo worlds do not include occupancy maps.  Generate one
map per world with SLAM, save the resulting `.yaml` and `.pgm` files here, and
then pass the saved YAML through the navigation launch `map` argument.
