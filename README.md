# PX4-Offboard-Tests

PX4 SITL offboard control test packages (ROS2 + uXRCE-DDS, using `px4_msgs`).

## Packages

- **`position_offboard_test`** (ament_python) — position-mode offboard example:
  arm → takeoff to 2 m → move 1 m forward → hover 5 s → land.
  ```bash
  ros2 run position_offboard_test offboard_control
  ```

- **`velocity_offboard_test`** (ament_cmake) — velocity-mode offboard control example.
  ```bash
  ros2 launch velocity_offboard_test velocity_controller.launch.py
  ```

## Build

Place the packages under a colcon workspace `src/`, then:

```bash
colcon build --packages-up-to position_offboard_test velocity_offboard_test
```

Requires **`px4_msgs`** matching your PX4 version (e.g. v1.16) and a running
uXRCE-DDS agent (`MicroXRCEAgent udp4 -p 8888`) bridging PX4's `/fmu/*` topics.
