#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

using namespace std::chrono_literals;

class VelocityController : public rclcpp::Node
{
public:
    VelocityController()
    : Node("velocity_controller"), tick_count_(0), last_command_("idle"), test_phase_("WAIT")
    {
        this->declare_parameter<int>("system_id", 1);
        system_id_ = this->get_parameter("system_id").as_int();

        std::string prefix = "vehicle" + std::to_string(system_id_) + "/fmu/";

        RCLCPP_INFO(this->get_logger(), "Configure velocity_controller (system_id: %d)", system_id_);

        auto qos = rclcpp::SensorDataQoS();

        // Publishers
        ocm_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            prefix + "in/offboard_control_mode", qos);
        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            prefix + "in/trajectory_setpoint", qos);
        cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            prefix + "in/vehicle_command", qos);

        // Subscribers
        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            prefix + "out/vehicle_status_v1", qos,
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                vehicle_status_ = *msg;
            });

        local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            prefix + "out/vehicle_local_position", qos,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                local_pos_ = *msg;
            });

        global_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            prefix + "out/vehicle_global_position", qos,
            [this](const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
                global_pos_ = *msg;
            });

        // 10Hz timer for heartbeat + flight sequence
        timer_ = this->create_wall_timer(100ms, std::bind(&VelocityController::timer_callback, this));
    }

private:
    void timer_callback()
    {
        publish_offboard_heartbeat();
        run_flight_sequence();
        print_status();
        ++tick_count_;
    }

    void publish_offboard_heartbeat()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = timestamp_us();
        msg.position = false;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.direct_actuator = false;
        ocm_pub_->publish(msg);
    }

    void run_flight_sequence()
    {
        constexpr float V  = 3.0f;   // 수평 테스트 속도 (m/s)
        constexpr float Vv = 2.0f;   // 수직 테스트 속도 (m/s)

        // ── 명령 이벤트 ──
        switch (tick_count_) {
        case 10:    send_arm();            break;  //  1.0s
        case 20:    send_takeoff(10.0);    break;  //  2.0s
        case 145:   send_offboard();       break;  // 14.5s
        case 705:   send_land();           break;  // 70.5s
        default: break;
        }

        // ── 속도 테스트 시퀀스 (각 5s 비행 + 3s hover) ──
        // offboard 진입 전 hover setpoint 선행 전송
        if      (tick_count_ >= 125 && tick_count_ < 155) {
            test_phase_ = "HOVER (pre-offboard)";
            send_velocity_setpoint(0.0f, 0.0f, 0.0f);
        }
        // 1) +Vx (body forward)  15.5s ~ 20.5s
        else if (tick_count_ >= 155 && tick_count_ < 205) {
            test_phase_ = "+Vx (forward)";
            send_velocity_setpoint(V, 0.0f, 0.0f);
        }
        // hover  20.5s ~ 23.5s
        else if (tick_count_ >= 205 && tick_count_ < 235) {
            test_phase_ = "HOVER";
            send_velocity_setpoint(0.0f, 0.0f, 0.0f);
        }
        // 2) -Vx (body backward)  23.5s ~ 28.5s
        else if (tick_count_ >= 235 && tick_count_ < 285) {
            test_phase_ = "-Vx (backward)";
            send_velocity_setpoint(-V, 0.0f, 0.0f);
        }
        // hover  28.5s ~ 31.5s
        else if (tick_count_ >= 285 && tick_count_ < 315) {
            test_phase_ = "HOVER";
            send_velocity_setpoint(0.0f, 0.0f, 0.0f);
        }
        // 3) +Vy (body right)  31.5s ~ 36.5s
        else if (tick_count_ >= 315 && tick_count_ < 365) {
            test_phase_ = "+Vy (right)";
            send_velocity_setpoint(0.0f, V, 0.0f);
        }
        // hover  36.5s ~ 39.5s
        else if (tick_count_ >= 365 && tick_count_ < 395) {
            test_phase_ = "HOVER";
            send_velocity_setpoint(0.0f, 0.0f, 0.0f);
        }
        // 4) -Vy (body left)  39.5s ~ 44.5s
        else if (tick_count_ >= 395 && tick_count_ < 445) {
            test_phase_ = "-Vy (left)";
            send_velocity_setpoint(0.0f, -V, 0.0f);
        }
        // hover  44.5s ~ 47.5s
        else if (tick_count_ >= 445 && tick_count_ < 475) {
            test_phase_ = "HOVER";
            send_velocity_setpoint(0.0f, 0.0f, 0.0f);
        }
        // 5) +Vz (NED down = 하강)  47.5s ~ 52.5s
        else if (tick_count_ >= 475 && tick_count_ < 525) {
            test_phase_ = "+Vz (down)";
            send_velocity_setpoint(0.0f, 0.0f, Vv);
        }
        // hover  52.5s ~ 55.5s
        else if (tick_count_ >= 525 && tick_count_ < 555) {
            test_phase_ = "HOVER";
            send_velocity_setpoint(0.0f, 0.0f, 0.0f);
        }
        // 6) -Vz (NED up = 상승)  55.5s ~ 60.5s
        else if (tick_count_ >= 555 && tick_count_ < 605) {
            test_phase_ = "-Vz (up)";
            send_velocity_setpoint(0.0f, 0.0f, -Vv);
        }
        // hover until land  60.5s ~ 70.5s
        else if (tick_count_ >= 605 && tick_count_ < 705) {
            test_phase_ = "HOVER (final)";
            send_velocity_setpoint(0.0f, 0.0f, 0.0f);
        }
    }

    void send_arm()
    {
        last_command_ = "ARM";
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.timestamp = timestamp_us();
        cmd.target_system = static_cast<uint8_t>(system_id_);
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        cmd.param1 = 1.0f;
        cmd.confirmation = 1;
        cmd.from_external = true;
        cmd_pub_->publish(cmd);
    }

    void send_takeoff(double altitude_m)
    {
        last_command_ = "TAKEOFF";
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.timestamp = timestamp_us();
        cmd.target_system = static_cast<uint8_t>(system_id_);
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
        cmd.param1 = -1.0f;
        cmd.param2 = 0.0f;
        cmd.param3 = 0.0f;
        cmd.param4 = local_pos_.heading;
        cmd.param5 = static_cast<double>(global_pos_.lat);
        cmd.param6 = static_cast<double>(global_pos_.lon);
        cmd.param7 = global_pos_.alt + static_cast<float>(altitude_m);
        cmd_pub_->publish(cmd);
    }

    void send_offboard()
    {
        last_command_ = "OFFBOARD";
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.timestamp = timestamp_us();
        cmd.target_system = static_cast<uint8_t>(system_id_);
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        cmd.param1 = 1.0f;
        cmd.param2 = 6.0f;  // PX4_CUSTOM_MAIN_MODE_OFFBOARD
        cmd.from_external = true;
        cmd_pub_->publish(cmd);
    }

    void send_land()
    {
        last_command_ = "LAND";
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.timestamp = timestamp_us();
        cmd.target_system = static_cast<uint8_t>(system_id_);
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
        cmd.from_external = true;
        cmd_pub_->publish(cmd);
    }

    void send_velocity_setpoint(float vx_body, float vy_body, float vz)
    {
        last_command_ = "VELOCITY";
        constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

        // Body frame → NED frame 변환 (heading 기준 회전)
        float yaw = local_pos_.heading;
        float vx_ned = vx_body * std::cos(yaw) - vy_body * std::sin(yaw);
        float vy_ned = vx_body * std::sin(yaw) + vy_body * std::cos(yaw);

        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = timestamp_us();
        msg.position = {NaN, NaN, NaN};
        msg.velocity = {vx_ned, vy_ned, vz};
        msg.yaw = NaN;
        msg.yawspeed = NaN;
        setpoint_pub_->publish(msg);
    }

    void print_status()
    {
        // Clear terminal every tick for live display
        std::printf("\033[2J\033[H");

        const char * arm_str =
            (vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
            ? "ARM" : "DISARM";

        RCLCPP_INFO(this->get_logger(),
            "nav_state: %u (%s) | tick: %d", vehicle_status_.nav_state, arm_str, tick_count_);
        RCLCPP_INFO(this->get_logger(),
            "NED pos: (%.2f, %.2f, %.2f)", local_pos_.x, local_pos_.y, local_pos_.z);
        RCLCPP_INFO(this->get_logger(),
            "NED vel: (%.2f, %.2f, %.2f)", local_pos_.vx, local_pos_.vy, local_pos_.vz);
        RCLCPP_INFO(this->get_logger(),
            "last command: %s", last_command_.c_str());
        RCLCPP_INFO(this->get_logger(),
            "test phase: %s", test_phase_.c_str());
    }

    uint64_t timestamp_us() const
    {
        return this->get_clock()->now().nanoseconds() / 1000;
    }

    // Parameters
    int system_id_;

    // State
    int tick_count_;
    std::string last_command_;
    std::string test_phase_;
    px4_msgs::msg::VehicleStatus vehicle_status_{};
    px4_msgs::msg::VehicleLocalPosition local_pos_{};
    px4_msgs::msg::VehicleGlobalPosition global_pos_{};

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_pos_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityController>());
    rclcpp::shutdown();
    return 0;
}
