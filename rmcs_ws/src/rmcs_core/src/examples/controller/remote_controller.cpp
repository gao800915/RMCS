#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::example {
class RemoteControllerExample
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    RemoteControllerExample()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
        register_input("/remote/joystick/left", remote_left_joystic_);
        register_input("/remote/joystick/right", remote_right_joystic_);
        register_input("/remote/switch/left", remote_left_switch_);
        register_input("/remote/switch/right", remote_right_switch_);
        //register_input("/example/M2006/angle", M2006velocity);
        //register_input("/example/M2006/control_torque", control_torque);
        register_output("/example/left2006/aim_velocity", left_motor_aim_velocity_);
        register_output("/example/right2006/aim_velocity", right_motor_aim_velocity_);
        
    }

    void update() override {
        using namespace rmcs_msgs;
        if ((*remote_left_switch_ == Switch::DOWN || *remote_left_switch_ == Switch::UNKNOWN)
            && (*remote_right_switch_ == Switch::DOWN || *remote_right_switch_ == Switch::UNKNOWN)) {
        *left_motor_aim_velocity_=0.0;
        *right_motor_aim_velocity_=0.0;
            // stop all !!
        } else {
          
            *left_motor_aim_velocity_ = 20 * remote_left_joystic_->x();
            *right_motor_aim_velocity_= 20 * remote_left_joystic_->x();
            if(count% 200 == 0){
            RCLCPP_INFO(get_logger(), "left_motor_aim_velocity_:%lf", *left_motor_aim_velocity_ );
            RCLCPP_INFO(get_logger(), "right_motor_aim_velocity_%lf", *right_motor_aim_velocity_ );
            //RCLCPP_INFO(get_logger(), "%lf",*M2006velocity);
            }
            count++;
            
        }
    }

private:
    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::Switch> remote_left_switch_;
    InputInterface<rmcs_msgs::Switch> remote_right_switch_;

    InputInterface<Eigen::Vector2d> remote_left_joystic_;
    InputInterface<Eigen::Vector2d> remote_right_joystic_;
    //InputInterface<double> M2006velocity;
    OutputInterface<double> left_motor_aim_velocity_;
    OutputInterface<double> right_motor_aim_velocity_;
    int count=0;
};

} // namespace rmcs_core::example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::example::RemoteControllerExample, rmcs_executor::Component)