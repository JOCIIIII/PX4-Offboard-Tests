# velocity_offboard_test

PX4 드론의 velocity offboard 제어를 테스트하는 ROS2 C++ 패키지.

ARM → TAKEOFF → OFFBOARD 진입 후, body frame 기준 Vx/Vy/Vz 각 축 +/- 방향으로 순차 비행하여 속도 제어를 검증한다.

## 의존성

- ROS2 Jazzy
- px4_msgs (branch: `release/1.16`)

## 빌드

```bash
source /opt/ros/jazzy/setup.bash
cd <RealGazebo-ROS2 워크스페이스>
colcon build --packages-select velocity_offboard_test
source install/setup.bash
```

## 실행

```bash
# RealGazebo 시뮬레이션이 실행 중인 상태에서
ros2 launch velocity_offboard_test velocity_controller.launch.py
```

### 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `system_id` | 1 | PX4 vehicle ID (토픽: `/vehicle{system_id}/fmu/...`) |

## 비행 시퀀스

| 시간 | 동작 |
|------|------|
| 1.0s | ARM |
| 2.0s | TAKEOFF (10m) |
| 12.5s | hover setpoint 시작 (offboard 진입 준비) |
| 14.5s | OFFBOARD 모드 전환 |
| 15.5s ~ 20.5s | **+Vx** (body forward) 3 m/s |
| 20.5s ~ 23.5s | hover |
| 23.5s ~ 28.5s | **-Vx** (body backward) 3 m/s |
| 28.5s ~ 31.5s | hover |
| 31.5s ~ 36.5s | **+Vy** (body right) 3 m/s |
| 36.5s ~ 39.5s | hover |
| 39.5s ~ 44.5s | **-Vy** (body left) 3 m/s |
| 44.5s ~ 47.5s | hover |
| 47.5s ~ 52.5s | **+Vz** (NED down, 하강) 2 m/s |
| 52.5s ~ 55.5s | hover |
| 55.5s ~ 60.5s | **-Vz** (NED up, 상승) 2 m/s |
| 60.5s ~ 70.5s | hover |
| 70.5s | LAND |

## 토픽

### Publish

| 토픽 | 메시지 타입 | 설명 |
|------|------------|------|
| `/vehicle{N}/fmu/in/offboard_control_mode` | `OffboardControlMode` | Offboard heartbeat (10Hz, velocity 모드) |
| `/vehicle{N}/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | 속도 setpoint (body→NED 변환 적용) |
| `/vehicle{N}/fmu/in/vehicle_command` | `VehicleCommand` | ARM, TAKEOFF, OFFBOARD, LAND 명령 |

### Subscribe

| 토픽 | 메시지 타입 | 설명 |
|------|------------|------|
| `/vehicle{N}/fmu/out/vehicle_status_v1` | `VehicleStatus` | arming/nav 상태 모니터링 |
| `/vehicle{N}/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | NED 위치/속도, heading |
| `/vehicle{N}/fmu/out/vehicle_global_position` | `VehicleGlobalPosition` | GPS 좌표 (takeoff용) |

## 좌표계

- **NED** (North-East-Down): +X=북, +Y=동, +Z=아래
- velocity setpoint는 body frame 입력을 heading 기준으로 NED 변환하여 전송:
  ```
  vx_ned = vx_body * cos(heading) - vy_body * sin(heading)
  vy_ned = vx_body * sin(heading) + vy_body * cos(heading)
  ```

## 터미널 출력

실행 중 터미널에 실시간으로 표시:
- `nav_state`, arming 상태, tick 카운트
- NED 위치 (x, y, z)
- NED 속도 (vx, vy, vz)
- 마지막 명령, 현재 테스트 단계
