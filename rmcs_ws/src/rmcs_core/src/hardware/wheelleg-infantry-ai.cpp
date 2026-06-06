
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <librmcs/agent/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dm_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"

namespace rmcs_core::hardware {

class WheelLegInfantry
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegInfantry()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<WheelLegInfantryCommand>(
                  get_component_name() + "_command", *this)) {
        register_output("/tf", tf_);

        gimbal_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/gimbal/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                gimbal_calibrate_subscription_callback(std::move(msg));
            });
        chassis_hips_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
            "/chassis/hip/calibrate", rclcpp::QoS{0},
            [this](std_msgs::msg::Int32::UniquePtr&& msg) {
                chassis_hip_calibrate_subscription_callback(std::move(msg));
            });

        // Preserve the current wheelleg runtime behavior: the top board code is migrated to the
        // new API, but the board itself is still not created until the hardware path is re-enabled.
        std::string bottom_board_serial;
        get_parameter_or("board_serial_bottom_board", bottom_board_serial, std::string{});
        bottom_board_ =
            std::make_shared<BottomBoard>(*this, *command_component_, bottom_board_serial);
    }

    WheelLegInfantry(const WheelLegInfantry&) = delete;
    WheelLegInfantry& operator=(const WheelLegInfantry&) = delete;
    WheelLegInfantry(WheelLegInfantry&&) = delete;
    WheelLegInfantry& operator=(WheelLegInfantry&&) = delete;

    ~WheelLegInfantry() override = default;

    void update() override {
        if (top_board_)
            top_board_->update();
        bottom_board_->update();
    }

    void command_update() {
        if (top_board_)
            top_board_->command_update();
        bottom_board_->command_update();
    }

private:
    void gimbal_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        if (bottom_board_) {
            RCLCPP_INFO(
                get_logger(), "[gimbal calibration] New yaw offset: %ld",
                bottom_board_->gimbal_yaw_motor_.calibrate_zero_point());
        }
        if (top_board_) {
            RCLCPP_INFO(
                get_logger(), "[gimbal calibration] New pitch offset: %ld",
                top_board_->gimbal_pitch_motor_.calibrate_zero_point());
        }
    }

    void chassis_hip_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] New left front hip offset: %ld",
            bottom_board_->chassis_hip_motors[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] New left back hip offset: %ld",
            bottom_board_->chassis_hip_motors[1].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] New right back hip offset: %ld",
            bottom_board_->chassis_hip_motors[2].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[chassis calibration] New right front hip offset: %ld",
            bottom_board_->chassis_hip_motors[3].calibrate_zero_point());
    }

    class WheelLegInfantryCommand : public rmcs_executor::Component {
    public:
        explicit WheelLegInfantryCommand(WheelLegInfantry& infantry)
            : infantry_(infantry) {}

        void update() override { infantry_.command_update(); }

        WheelLegInfantry& infantry_;
    };

    class TopBoard final : private librmcs::agent::CBoard {
    public:
        friend class WheelLegInfantry;

        explicit TopBoard(
            WheelLegInfantry& infantry, WheelLegInfantryCommand& infantry_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , imu_(1000, 0.2, 0.0)
            , tf_(infantry.tf_)
            , gimbal_pitch_motor_(infantry, infantry_command, "/gimbal/pitch")
            , gimbal_left_friction_(infantry, infantry_command, "/gimbal/left_friction")
            , gimbal_right_friction_(infantry, infantry_command, "/gimbal/right_friction") {
            gimbal_pitch_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei10}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            infantry.get_parameter("pitch_motor_zero_point").as_int())));

            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reduction_ratio(1.0));
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reduction_ratio(1.0)
                    .set_reversed());

            infantry.register_output("/gimbal/yaw/velocity", gimbal_yaw_velocity_imu_);
            infantry.register_output("/gimbal/pitch/velocity", gimbal_pitch_velocity_imu_);

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                return std::make_tuple(x, y, z);
            });
        }

        TopBoard(const TopBoard&) = delete;
        TopBoard& operator=(const TopBoard&) = delete;
        TopBoard(TopBoard&&) = delete;
        TopBoard& operator=(TopBoard&&) = delete;

        ~TopBoard() override = default;

        void update() {
            imu_.update_status();
            const Eigen::Quaterniond gimbal_imu_pose{
                imu_.q0(), imu_.q1(), imu_.q2(), imu_.q3()};

            tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                gimbal_imu_pose.conjugate());

            gimbal_pitch_motor_.update_status();
            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();

            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                gimbal_pitch_motor_.angle());

            *gimbal_yaw_velocity_imu_ = imu_.gz();
            *gimbal_pitch_velocity_imu_ = imu_.gy();
        }

        void command_update() {
            auto builder = start_transmit();

            builder.can1_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       gimbal_left_friction_.generate_command(),
                                       gimbal_right_friction_.generate_command(),
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });

            auto pitch_command =
                gimbal_pitch_motor_.generate_velocity_command(gimbal_pitch_motor_.control_velocity());
            builder.can2_transmit({
                .can_id = 0x141,
                .can_data = pitch_command.as_bytes(),
            });
        }

    private:
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            if (data.can_id == 0x201) {
                gimbal_left_friction_.store_status(data.can_data);
            } else if (data.can_id == 0x202) {
                gimbal_right_friction_.store_status(data.can_data);
            }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            if (data.can_id == 0x141)
                gimbal_pitch_motor_.store_status(data.can_data);
        }

        void accelerometer_receive_callback(
            const librmcs::data::AccelerometerDataView& data) override {
            imu_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
            imu_.store_gyroscope_status(data.x, data.y, data.z);
        }

        device::Bmi088 imu_;

        OutputInterface<rmcs_description::Tf>& tf_;

        OutputInterface<double> gimbal_yaw_velocity_imu_;
        OutputInterface<double> gimbal_pitch_velocity_imu_;

        device::LkMotor gimbal_pitch_motor_;
        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;
    };

    class BottomBoard final : private librmcs::agent::CBoard {
    public:
        friend class WheelLegInfantry;

        explicit BottomBoard(
            WheelLegInfantry& infantry, WheelLegInfantryCommand& infantry_command,
            std::string_view board_serial = {})
            : librmcs::agent::CBoard(board_serial)
            , dr16_(infantry)
            , imu_(1000, 0.2, 0.0)
            , tf_(infantry.tf_)
            , chassis_wheel_motors_(
                  {infantry, infantry_command, "/chassis/left_wheel"},
                  {infantry, infantry_command, "/chassis/right_wheel"})
            , chassis_hip_motors(
                  {infantry, infantry_command, "/chassis/left_front_hip"},
                  {infantry, infantry_command, "/chassis/left_back_hip"},
                  {infantry, infantry_command, "/chassis/right_back_hip"},
                  {infantry, infantry_command, "/chassis/right_front_hip"})
            , gimbal_yaw_motor_(infantry, infantry_command, "/gimbal/yaw")
            , bullet_feeder_motor_(infantry, infantry_command, "/gimbal/bullet_feeder") {
            chassis_wheel_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reduction_ratio(268.0 / 17.0)
                    .enable_multi_turn_angle());
            chassis_wheel_motors_[1].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reduction_ratio(268.0 / 17.0)
                    .enable_multi_turn_angle()
                    .set_reversed());

            chassis_hip_motors[0].configure(
                device::DmMotor::Config{device::DmMotor::Type::kDM8009}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            infantry.get_parameter("left_front_hip_motor_zero_point").as_int()))
                    .set_reversed());
            chassis_hip_motors[1].configure(
                device::DmMotor::Config{device::DmMotor::Type::kDM8009}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            infantry.get_parameter("left_back_hip_motor_zero_point").as_int()))
                    .set_reversed());
            chassis_hip_motors[2].configure(
                device::DmMotor::Config{device::DmMotor::Type::kDM8009}.set_encoder_zero_point(
                    static_cast<int>(
                        infantry.get_parameter("right_back_hip_motor_zero_point").as_int())));
            chassis_hip_motors[3].configure(
                device::DmMotor::Config{device::DmMotor::Type::kDM8009}.set_encoder_zero_point(
                    static_cast<int>(
                        infantry.get_parameter("right_front_hip_motor_zero_point").as_int())));

            gimbal_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}
                    .set_encoder_zero_point(
                        static_cast<int>(
                            infantry.get_parameter("yaw_motor_zero_point").as_int())));
            bullet_feeder_motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM2006}
                    .enable_multi_turn_angle()
                    .set_reversed()
                    .set_reduction_ratio(19 * 2));

            infantry.register_output("/referee/serial", referee_serial_);

            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                start_transmit().uart1_transmit({
                    .uart_data = std::span<const std::byte>{buffer, size},
                });
                return size;
            };

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                return std::make_tuple(x, y, z);
            });

            infantry.register_output(
                "/chassis/x_axis/acceleration", chassis_x_axis_acceleration_imu_);
            infantry.register_output(
                "/chassis/z_axis/acceleration", chassis_z_axis_acceleration_imu_);

            infantry.register_output("/chassis/yaw/velocity", chassis_yaw_velocity_imu_);
            infantry.register_output("/chassis/pitch/velocity", chassis_pitch_velocity_imu_);
            infantry.register_output("/chassis/roll/velocity", chassis_roll_velocity_imu_);

            infantry.register_output("/chassis/yaw/angle", chassis_yaw_angle_imu_);
            infantry.register_output("/chassis/pitch/angle", chassis_pitch_angle_imu_);
            infantry.register_output("/chassis/roll/angle", chassis_roll_angle_imu_);
        }

        BottomBoard(const BottomBoard&) = delete;
        BottomBoard& operator=(const BottomBoard&) = delete;
        BottomBoard(BottomBoard&&) = delete;
        BottomBoard& operator=(BottomBoard&&) = delete;

        ~BottomBoard() override = default;

        void update() {
            dr16_.update_status();

            update_imu();

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();

            for (auto& motor : chassis_hip_motors)
                motor.update_status();

            gimbal_yaw_motor_.update_status();
            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());

            bullet_feeder_motor_.update_status();
        }

        void command_update() {
            auto builder = start_transmit();

            builder.can1_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       chassis_wheel_motors_[0].generate_command(),
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });

            auto left_front_hip_command = chassis_hip_motors[0].generate_command();
            builder.can1_transmit({
                .can_id = 0x01,
                .can_data = left_front_hip_command.as_bytes(),
            });

            auto left_back_hip_command = chassis_hip_motors[1].generate_command();
            builder.can1_transmit({
                .can_id = 0x02,
                .can_data = left_back_hip_command.as_bytes(),
            });

            builder.can2_transmit({
                .can_id = 0x200,
                .can_data =
                    device::CanPacket8{
                                       device::CanPacket8::PaddingQuarter{},
                                       chassis_wheel_motors_[1].generate_command(),
                                       device::CanPacket8::PaddingQuarter{},
                                       device::CanPacket8::PaddingQuarter{},
                                       }
                        .as_bytes(),
            });

            auto right_back_hip_command = chassis_hip_motors[2].generate_command();
            builder.can2_transmit({
                .can_id = 0x03,
                .can_data = right_back_hip_command.as_bytes(),
            });

            auto right_front_hip_command = chassis_hip_motors[3].generate_command();
            builder.can2_transmit({
                .can_id = 0x04,
                .can_data = right_front_hip_command.as_bytes(),
            });
        }

        void update_imu() {
            imu_.update_status();

            *chassis_yaw_angle_imu_ = std::atan2(
                2.0 * (imu_.q0() * imu_.q3() + imu_.q1() * imu_.q2()),
                2.0 * (imu_.q0() * imu_.q0() + imu_.q1() * imu_.q1()) - 1.0);
            *chassis_pitch_angle_imu_ =
                std::asin(-2.0 * (imu_.q1() * imu_.q3() - imu_.q0() * imu_.q2()));
            *chassis_roll_angle_imu_ = std::atan2(
                2.0 * (imu_.q0() * imu_.q1() + imu_.q2() * imu_.q3()),
                2.0 * (imu_.q0() * imu_.q0() + imu_.q3() * imu_.q3()) - 1.0);

            *chassis_yaw_velocity_imu_ = imu_.gz();
            *chassis_pitch_velocity_imu_ = imu_.gy();
            *chassis_roll_velocity_imu_ = imu_.gx();

            *chassis_x_axis_acceleration_imu_ = imu_.ax();
            *chassis_z_axis_acceleration_imu_ = imu_.az();
        }

    private:
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            if (data.can_id == 0x201) {
                chassis_wheel_motors_[0].store_status(data.can_data);
            } else if (data.can_id == 0x01) {
                chassis_hip_motors[0].store_status(data.can_data);
            } else if (data.can_id == 0x02) {
                chassis_hip_motors[1].store_status(data.can_data);
            }
        }

        void can2_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            if (data.can_id == 0x202) {
                chassis_wheel_motors_[1].store_status(data.can_data);
            } else if (data.can_id == 0x03) {
                chassis_hip_motors[2].store_status(data.can_data);
            } else if (data.can_id == 0x04) {
                chassis_hip_motors[3].store_status(data.can_data);
            }
        }

        void uart1_receive_callback(const librmcs::data::UartDataView& data) override {
            const auto* uart_data = data.uart_data.data();
            referee_ring_buffer_receive_.emplace_back_n(
                [&uart_data](std::byte* storage) noexcept { *storage = *uart_data++; },
                data.uart_data.size());
        }

        void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        }

        void accelerometer_receive_callback(
            const librmcs::data::AccelerometerDataView& data) override {
            imu_.store_accelerometer_status(data.x, data.y, data.z);
        }

        void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
            imu_.store_gyroscope_status(data.x, data.y, data.z);
        }

        device::Dr16 dr16_;
        device::Bmi088 imu_;

        OutputInterface<double> chassis_x_axis_acceleration_imu_;
        OutputInterface<double> chassis_z_axis_acceleration_imu_;

        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> chassis_pitch_velocity_imu_;
        OutputInterface<double> chassis_roll_velocity_imu_;

        OutputInterface<double> chassis_yaw_angle_imu_;
        OutputInterface<double> chassis_pitch_angle_imu_;
        OutputInterface<double> chassis_roll_angle_imu_;

        OutputInterface<rmcs_description::Tf>& tf_;

        device::DjiMotor chassis_wheel_motors_[2];
        device::DmMotor chassis_hip_motors[4];

        device::LkMotor gimbal_yaw_motor_;
        device::DjiMotor bullet_feeder_motor_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;
    };

    OutputInterface<rmcs_description::Tf> tf_;

    std::shared_ptr<WheelLegInfantryCommand> command_component_;
    std::shared_ptr<TopBoard> top_board_;
    std::shared_ptr<BottomBoard> bottom_board_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gimbal_calibrate_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr chassis_hips_calibrate_subscription_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::WheelLegInfantry, rmcs_executor::Component)
