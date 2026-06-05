#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numbers>
#include <span>
#include <string>
#include <utility>

#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"

namespace rmcs_core::hardware::device {

class DmMotor {
public:
    enum class Type : uint8_t { kDM8009 };

    struct Config {
        explicit Config(Type motor_type)
            : motor_type(motor_type) {}

        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }

        Type motor_type;
        int encoder_zero_point = 0;
        bool reversed = false;
        bool multi_turn_angle_enabled = false;
    };

    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix)
        : angle_(0.0)
        , velocity_(0.0)
        , torque_(0.0) {
        status_component.register_output(name_prefix + "/angle", angle_output_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_output_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_output_, 0.0);

        command_component.register_input(name_prefix + "/control_angle", control_angle_, false);
        command_component.register_input(
            name_prefix + "/control_velocity", control_velocity_, false);
        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);
    }

    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : DmMotor(status_component, command_component, name_prefix) {
        configure(config);
    }

    void configure(const Config& config) {
        multi_turn_encoder_count_ = 0;
        last_raw_angle_ = 0;
        motor_state_ = 0;

        const double reduction_ratio = 1.0;
        if (config.motor_type == Type::kDM8009) {
            raw_angle_max_ = 65535;
            max_velocity_ = 45.0;
            max_torque_ = 54.0;
            max_angle_ = std::numbers::pi;
            max_kp_ = 500.0;
            max_kd_ = 5.0;
        }

        encoder_zero_point_ = config.encoder_zero_point % raw_angle_max_;
        if (encoder_zero_point_ < 0)
            encoder_zero_point_ += raw_angle_max_;

        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;

        const double sign = config.reversed ? -1.0 : 1.0;

        status_angle_to_angle_coefficient_ = sign / raw_angle_max_ * 2 * std::numbers::pi;
        angle_to_command_angle_coefficient_ = sign * reduction_ratio * kRadToDeg;

        status_velocity_to_velocity_coefficient_ = sign / reduction_ratio;
        velocity_to_command_velocity_coefficient_ = sign * reduction_ratio;

        status_torque_to_torque_coefficient_ = sign * reduction_ratio;
        torque_to_command_torque_coefficient_ = 1.0 / status_torque_to_torque_coefficient_;

        *max_torque_output_ = max_torque();
    }

    void store_status(std::span<const std::byte> can_data) {
        if (can_data.size() != 8) [[unlikely]]
            return;

        can_data_.store(CanPacket8{can_data}, std::memory_order_relaxed);
    }

    void update_status() {
        const struct [[gnu::packed]] {
            uint8_t id_state;
            uint8_t encoder_high;
            uint8_t encoder_low;
            uint8_t velocity_high;
            uint8_t velocity_low_torque_high;
            uint8_t torque_low;
            uint8_t mos_temperature;
            uint8_t rotor_temperature;
        } feedback alignas(CanPacket8) =
            std::bit_cast<decltype(feedback)>(can_data_.load(std::memory_order_relaxed));

        id_ = feedback.id_state & 0x0F;
        motor_state_ = feedback.id_state >> 4;

        const uint16_t encoder =
            (static_cast<uint16_t>(feedback.encoder_high) << 8) | feedback.encoder_low;
        const uint16_t raw_velocity =
            (static_cast<uint16_t>(feedback.velocity_high) << 4)
            | (feedback.velocity_low_torque_high >> 4);
        const uint16_t raw_torque =
            (static_cast<uint16_t>(feedback.velocity_low_torque_high & 0x0F) << 8)
            | feedback.torque_low;

        mos_temperature_ = static_cast<double>(feedback.mos_temperature);
        rotor_temperature_ = static_cast<double>(feedback.rotor_temperature);

        const int raw_angle = static_cast<int>(encoder);
        position_ = uint_to_double(encoder, -max_angle_, max_angle_, 16);

        int calibrated_raw_angle = encoder - encoder_zero_point_;
        if (calibrated_raw_angle < 0)
            calibrated_raw_angle += raw_angle_max_;

        if (!multi_turn_angle_enabled_) {
            angle_ = status_angle_to_angle_coefficient_ * static_cast<double>(calibrated_raw_angle);
            if (angle_ < 0)
                angle_ += 2 * std::numbers::pi;
        } else {
            auto diff = (calibrated_raw_angle - multi_turn_encoder_count_) & (raw_angle_max_ - 1);
            if (diff > (raw_angle_max_ >> 1))
                diff -= raw_angle_max_;

            multi_turn_encoder_count_ += diff;
            angle_ = status_angle_to_angle_coefficient_
                * static_cast<double>(multi_turn_encoder_count_);
        }

        last_raw_angle_ = raw_angle;

        velocity_ = status_velocity_to_velocity_coefficient_
                  * uint_to_double(raw_velocity, -max_velocity_, max_velocity_, 12);
        torque_ = status_torque_to_torque_coefficient_
                * uint_to_double(raw_torque, -max_torque_, max_torque_, 12);

        *angle_output_ = angle();
        *velocity_output_ = velocity();
        *torque_output_ = torque();
    }

    double control_torque() const {
        if (control_torque_.ready()) [[likely]]
            return *control_torque_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    double control_angle() const {
        if (control_angle_.ready()) [[likely]]
            return *control_angle_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    double control_velocity() const {
        if (control_velocity_.ready()) [[likely]]
            return *control_velocity_;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    constexpr static CanPacket8 generate_enable_command() {
        const struct [[gnu::packed]] {
            uint8_t message[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
        } command alignas(CanPacket8){};
        return std::bit_cast<CanPacket8>(command);
    }

    constexpr static CanPacket8 generate_clear_error_command() {
        const struct [[gnu::packed]] {
            uint8_t message[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
        } command alignas(CanPacket8){};
        return std::bit_cast<CanPacket8>(command);
    }

    CanPacket8 generate_disable_command() const {
        return generate_mit_command(0.0, 0.0, 0.0, 0.0, 0.0);
    }

    CanPacket8 generate_torque_command(double control_torque) const {
        if (std::isnan(control_torque))
            return generate_disable_command();

        return generate_mit_command(0.0, 0.0, to_command_torque(control_torque), 0.0, 0.0);
    }

    CanPacket8 generate_angle_command(
        double control_angle,
        double control_kp = 60.0,
        double control_kd = 2.0) const {
        if (std::isnan(control_angle))
            return generate_disable_command();

        return generate_mit_command(control_angle, 0.0, 0.0, control_kp, control_kd);
    }

    CanPacket8 generate_command() const {
        if (motor_state_ == 0) {
            return generate_enable_command();
        } else if (motor_state_ == 1) {
            return generate_torque_command(control_torque());
        } else {
            return generate_clear_error_command();
        }
    }

    int64_t calibrate_zero_point() {
        multi_turn_encoder_count_ = 0;
        encoder_zero_point_ = last_raw_angle_;
        return encoder_zero_point_;
    }

    uint8_t error_state() const { return motor_state_; }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return max_torque_; }
    double position() const { return position_; }
    double mos_temperature() const { return mos_temperature_; }
    double rotor_temperature() const { return rotor_temperature_; }

    CanPacket8 generate_mit_command(
        double control_angle,
        double control_velocity,
        double control_torque,
        double control_kp,
        double control_kd) const {
        control_angle = std::clamp(control_angle, -max_angle_, max_angle_);
        control_velocity = std::clamp(control_velocity, -max_velocity_, max_velocity_);
        control_torque = std::clamp(control_torque, -max_torque_, max_torque_);
        control_kp = std::clamp(control_kp, 0.0, max_kp_);
        control_kd = std::clamp(control_kd, 0.0, max_kd_);

        const uint16_t angle = double_to_uint(control_angle, -max_angle_, max_angle_, 16);
        const uint16_t velocity = double_to_uint(control_velocity, -max_velocity_, max_velocity_, 12);
        const uint16_t torque = double_to_uint(control_torque, -max_torque_, max_torque_, 12);
        const uint16_t kp = double_to_uint(control_kp, 0.0, max_kp_, 12);
        const uint16_t kd = double_to_uint(control_kd, 0.0, max_kd_, 12);

        const struct [[gnu::packed]] {
            uint8_t byte0;
            uint8_t byte1;
            uint8_t byte2;
            uint8_t byte3;
            uint8_t byte4;
            uint8_t byte5;
            uint8_t byte6;
            uint8_t byte7;
        } command alignas(CanPacket8){
            .byte0 = static_cast<uint8_t>(angle >> 8),
            .byte1 = static_cast<uint8_t>(angle),
            .byte2 = static_cast<uint8_t>(velocity >> 4),
            .byte3 = static_cast<uint8_t>(((velocity & 0x0F) << 4) | (kp >> 8)),
            .byte4 = static_cast<uint8_t>(kp),
            .byte5 = static_cast<uint8_t>(kd >> 4),
            .byte6 = static_cast<uint8_t>(((kd & 0x0F) << 4) | (torque >> 8)),
            .byte7 = static_cast<uint8_t>(torque),
        };

        return std::bit_cast<CanPacket8>(command);
    }

private:
    static double uint_to_double(int x_int, double x_min, double x_max, int bits) {
        const double span = x_max - x_min;
        const double offset = x_min;
        return static_cast<double>(x_int) * span / static_cast<double>((1u << bits) - 1u)
             + offset;
    }

    static int double_to_uint(double x_double, double x_min, double x_max, int bits) {
        const double span = x_max - x_min;
        const double offset = x_min;
        return static_cast<int>(
            (x_double - offset) * static_cast<double>((1u << bits) - 1u) / span);
    }

    double to_command_torque(double torque) const {
        return torque_to_command_torque_coefficient_ * torque;
    }

    double to_command_velocity(double velocity) const {
        return velocity_to_command_velocity_coefficient_ * velocity;
    }

    std::atomic<CanPacket8> can_data_{CanPacket8{0}};

    static constexpr double kDegToRad = std::numbers::pi / 180.0;
    static constexpr double kRadToDeg = 180.0 / std::numbers::pi;

    int raw_angle_max_{};
    int encoder_zero_point_{};

    bool multi_turn_angle_enabled_{false};
    int64_t multi_turn_encoder_count_{0};
    int last_raw_angle_{0};

    double status_angle_to_angle_coefficient_{};
    double angle_to_command_angle_coefficient_{};
    double status_velocity_to_velocity_coefficient_{};
    double velocity_to_command_velocity_coefficient_{};
    double status_torque_to_torque_coefficient_{};
    double torque_to_command_torque_coefficient_{};

    uint8_t id_{0};
    uint8_t motor_state_{0};

    double angle_{0.0};
    double velocity_{0.0};
    double torque_{0.0};
    double position_{0.0};

    double max_angle_{0.0};
    double max_torque_{0.0};
    double max_velocity_{0.0};
    double max_kp_{0.0};
    double max_kd_{0.0};

    double mos_temperature_{0.0};
    double rotor_temperature_{0.0};

    rmcs_executor::Component::OutputInterface<double> angle_output_;
    rmcs_executor::Component::OutputInterface<double> velocity_output_;
    rmcs_executor::Component::OutputInterface<double> torque_output_;
    rmcs_executor::Component::OutputInterface<double> max_torque_output_;

    rmcs_executor::Component::InputInterface<double> control_angle_;
    rmcs_executor::Component::InputInterface<double> control_velocity_;
    rmcs_executor::Component::InputInterface<double> control_torque_;
};

} // namespace rmcs_core::hardware::device
