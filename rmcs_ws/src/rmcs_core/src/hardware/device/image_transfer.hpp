#pragma once

#include <eigen3/Eigen/Dense>
#include <librmcs/device/image_transfer.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/vtswitch.hpp>

namespace rmcs_core::hardware::device {

class ImageTrans : public librmcs::device::ImageTrans {
public:
    explicit ImageTrans(rmcs_executor::Component& component) {
        component.register_output(
            "/remote/joystick/right", joystick_right_, Eigen::Vector2d::Zero());
        component.register_output("/remote/joystick/left", joystick_left_, Eigen::Vector2d::Zero());

        component.register_output(
            "/remote/switch/state", switch_state_, rmcs_msgs::VTSwitch::UNKNOWN);

        component.register_output(
            "/remote/pause/state", pause_,0);

        component.register_output(
            "/remote/trigger/state", trigger_,false);

        component.register_output(
            "/remote/custom/left", custom_key_left_,false);

        component.register_output(
            "/remote/custom/right", custom_key_right_,false);

        component.register_output(
            "/remote/thumbwheel/state", thumbwheel_,0);

        component.register_output(
            "/remote/mouse/velocity", mouse_velocity_, Eigen::Vector2d::Zero());
        component.register_output("/remote/mouse/mouse_wheel", mouse_wheel_);

        component.register_output("/remote/mouse", mouse_);
        std::memset(&*mouse_, 0, sizeof(*mouse_));
        component.register_output("/remote/keyboard", keyboard_);
        std::memset(&*keyboard_, 0, sizeof(*keyboard_));


        // Simulate the rotary knob as a switch, with anti-shake algorithm.
        
    }

    void update_status() {
        librmcs::device::ImageTrans::update_status();

        *joystick_right_ = joystick_right();
        *joystick_left_ = joystick_left();

        *switch_state_ = switch_state();

        *pause_=pause();

        *mouse_velocity_ = mouse_velocity();
        *mouse_wheel_ = mouse_wheel();

        *mouse_ = mouse();
        *keyboard_ = keyboard();
        *thumbwheel_=thumbwheel();
        *trigger_=trigger();

        *custom_key_left_=custom_key_left();
        *custom_key_right_=custom_key_right();
        
    }

    Eigen::Vector2d joystick_right() const {
        return to_eigen_vector(librmcs::device::ImageTrans::joystick_right());
    }
    Eigen::Vector2d joystick_left() const {
        return to_eigen_vector(librmcs::device::ImageTrans::joystick_left());
    }

    rmcs_msgs::VTSwitch switch_state() const {
        return std::bit_cast<rmcs_msgs::VTSwitch>(librmcs::device::ImageTrans::switch_state());
    }

    Eigen::Vector2d mouse_velocity() const {
        return to_eigen_vector(librmcs::device::ImageTrans::mouse_velocity());
    }

    rmcs_msgs::Mouse mouse() const {
        return std::bit_cast<rmcs_msgs::Mouse>(librmcs::device::ImageTrans::mouse());
    }
    rmcs_msgs::Keyboard keyboard() const {
        return std::bit_cast<rmcs_msgs::Keyboard>(librmcs::device::ImageTrans::keyboard());
    }

private:
    static Eigen::Vector2d to_eigen_vector(Vector vector) { return {vector.x, vector.y}; }

    

    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_right_;
    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> joystick_left_;

    rmcs_executor::Component::OutputInterface<rmcs_msgs::VTSwitch> switch_state_;

    rmcs_executor::Component::OutputInterface<bool> pause_;
    rmcs_executor::Component::OutputInterface<bool> custom_key_left_;
    rmcs_executor::Component::OutputInterface<bool> custom_key_right_;
    rmcs_executor::Component::OutputInterface<bool> trigger_;

    rmcs_executor::Component::OutputInterface<Eigen::Vector2d> mouse_velocity_;
    rmcs_executor::Component::OutputInterface<double> mouse_wheel_;
    rmcs_executor::Component::OutputInterface<double> thumbwheel_;

    rmcs_executor::Component::OutputInterface<rmcs_msgs::Mouse> mouse_;
    rmcs_executor::Component::OutputInterface<rmcs_msgs::Keyboard> keyboard_;
    

};

} // namespace rmcs_core::hardware::device