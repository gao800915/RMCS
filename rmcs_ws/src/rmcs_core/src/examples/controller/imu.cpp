#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>


namespace rmcs_core::example {
class imu
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    imu()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
       
        register_input("/dragon/angle/torque", angle_torgue);
        //register_input("/example/M2006/angle", M2006velocity);
        //register_input("/example/M2006/control_torque", control_torque);
        register_input("/example/left2006/control_torque_base", left_control_torque_base);
        register_input("/example/right2006/control_torque_base", right_control_torque_base);
        register_output("/example/left2006/control_torque", left_control_torque_);
        register_output("/example/right2006/control_torque", right_control_torque_);
    }

    void update() override {
        using namespace rmcs_msgs;
      //   RCLCPP_INFO(get_logger(), "left_control_data:%lf",*left_control_torque_);
      //    RCLCPP_INFO(get_logger(), "right_control_data:%lf",*right_control_torque_);
        *left_control_torque_=*left_control_torque_base+0.2*(*angle_torgue);
        *right_control_torque_=*right_control_torque_base-0.2*(*angle_torgue);
        }
    

private:
    rclcpp::Logger logger_;

    InputInterface<double> angle_torgue;
    InputInterface<double> left_control_torque_base;
    InputInterface<double> right_control_torque_base;
    //InputInterface<double> M2006velocity;

    OutputInterface<double> left_motor_aim_velocity_;
    OutputInterface<double> right_motor_aim_velocity_;
    OutputInterface<double> left_control_torque_;
    OutputInterface<double> right_control_torque_;

};

} // namespace rmcs_core::example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::example::imu, rmcs_executor::Component)