#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus


class OffboardControl(Node):
    """PX4 Offboard Position Control - 1m Forward Movement"""

    def __init__(self):
        super().__init__('offboard_control')

        # QoS 설정 (PX4와 통신용)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)

        # 상태 변수
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0

        # 목표 위치 (초기화 후 설정)
        self.start_position_set = False
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_z = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -2.0  # 기본 고도 2m (NED 좌표계에서 -Z가 위)

        # 상태 머신
        self.state = 'INIT'  # INIT -> TAKEOFF -> FORWARD -> HOVER -> LAND

        # 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Offboard Control Node Started')
        self.get_logger().info('Mission: Takeoff -> Move 1m Forward -> Hover')

    def vehicle_local_position_callback(self, msg):
        """현재 로컬 위치 업데이트"""
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        """차량 상태 업데이트"""
        self.vehicle_status = msg

    def arm(self):
        """드론 Arm"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """드론 Disarm"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """오프보드 모드 진입"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode command sent')

    def land(self):
        """착륙 명령"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land command sent')

    def publish_offboard_control_mode(self):
        """오프보드 제어 모드 퍼블리시 (Position 모드)"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
        """위치 목표점 퍼블리시 (NED 좌표계)"""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, **params):
        """Vehicle Command 퍼블리시"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def is_at_position(self, target_x, target_y, target_z, tolerance=0.2):
        """목표 위치 도달 여부 확인"""
        dx = abs(self.vehicle_local_position.x - target_x)
        dy = abs(self.vehicle_local_position.y - target_y)
        dz = abs(self.vehicle_local_position.z - target_z)
        return dx < tolerance and dy < tolerance and dz < tolerance

    def timer_callback(self):
        """메인 제어 루프 (10Hz)"""

        # 오프보드 제어 모드 지속적으로 퍼블리시 (필수)
        self.publish_offboard_control_mode()

        # 초기 시작 위치 저장
        if not self.start_position_set and self.vehicle_local_position.timestamp > 0:
            self.start_x = self.vehicle_local_position.x
            self.start_y = self.vehicle_local_position.y
            self.start_z = self.vehicle_local_position.z
            self.start_position_set = True
            self.get_logger().info(f'Start position: x={self.start_x:.2f}, y={self.start_y:.2f}, z={self.start_z:.2f}')

        # 상태 머신
        if self.state == 'INIT':
            # 오프보드 모드 진입 전 최소 setpoint 전송 (PX4 요구사항)
            self.publish_trajectory_setpoint(
                self.start_x, self.start_y, -2.0)  # 이륙 고도 2m

            if self.offboard_setpoint_counter >= 10:
                self.engage_offboard_mode()
                self.arm()
                self.state = 'TAKEOFF'
                self.get_logger().info('State: TAKEOFF - Ascending to 2m altitude')

            self.offboard_setpoint_counter += 1

        elif self.state == 'TAKEOFF':
            # 이륙: 현재 x, y 유지하면서 고도 2m로 상승
            takeoff_x = self.start_x
            takeoff_y = self.start_y
            takeoff_z = -2.0  # 2m 고도 (NED)

            self.publish_trajectory_setpoint(takeoff_x, takeoff_y, takeoff_z)

            if self.is_at_position(takeoff_x, takeoff_y, takeoff_z):
                self.state = 'FORWARD'
                # 목표 위치: X축(North 방향)으로 1m 전진
                self.target_x = takeoff_x + 1.0
                self.target_y = takeoff_y
                self.target_z = takeoff_z
                self.get_logger().info(f'State: FORWARD - Moving 1m forward to x={self.target_x:.2f}')

        elif self.state == 'FORWARD':
            # 1m 전진
            self.publish_trajectory_setpoint(self.target_x, self.target_y, self.target_z)

            if self.is_at_position(self.target_x, self.target_y, self.target_z):
                self.state = 'HOVER'
                self.hover_counter = 0
                self.get_logger().info('State: HOVER - Reached target position!')
                self.get_logger().info(f'Current position: x={self.vehicle_local_position.x:.2f}, '
                                      f'y={self.vehicle_local_position.y:.2f}, '
                                      f'z={self.vehicle_local_position.z:.2f}')

        elif self.state == 'HOVER':
            # 목표 위치에서 호버링 (5초 후 착륙)
            self.publish_trajectory_setpoint(self.target_x, self.target_y, self.target_z)
            self.hover_counter += 1

            if self.hover_counter >= 50:  # 5초 (10Hz * 50)
                self.state = 'LAND'
                self.land()
                self.get_logger().info('State: LAND - Mission complete, landing...')

        elif self.state == 'LAND':
            # 착륙 중 - 오프보드 모드 유지하며 현재 위치 전송
            self.publish_trajectory_setpoint(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z)


def main(args=None):
    print('Starting PX4 Offboard Position Control Node...')
    print('Mission: Takeoff (2m) -> Move 1m Forward -> Hover (5s) -> Land')
    print('')

    rclpy.init(args=args)
    offboard_control = OffboardControl()

    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        print('\nKeyboard interrupt, shutting down...')
    finally:
        offboard_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
