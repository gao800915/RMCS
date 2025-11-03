#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>


namespace rmcs_core::example {
class pitchcontrolltorque
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    pitchcontrolltorque()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
   
        register_input("/target/pitch/left/basic/torque", target_left_control_torque_base);
        register_input("//target/pitch/right/basic/torque", target_right_control_torque_base);
        register_input("roll/angle/basictorque", angle_torgue);

        register_output("/example/left2006/control_torque", target_left_control_torque_);
        register_output("/example/right2006/control_torque", target_right_control_torque_);
    }

    void update() override {
        using namespace rmcs_msgs;
        *target_left_control_torque_=*target_left_control_torque_base+0.2*(*angle_torgue);
        *target_right_control_torque_=*target_right_control_torque_base-0.2*(*angle_torgue);
        }
    

private:
    rclcpp::Logger logger_;

    InputInterface<double> angle_torgue;
    
    InputInterface<double> target_left_control_torque_base;
    InputInterface<double> target_right_control_torque_base;

    
    OutputInterface<double> target_left_control_torque_;
    OutputInterface<double> target_right_control_torque_;



};

} // namespace rmcs_core::example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::example::pitchcontrolltorque, rmcs_executor::Component)