#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#define fc 10.0 //Corner Frequency
#define fs 1000.0 //Sampling Frequency

namespace rmcs_core::example {
class info
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    info()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
         {
             register_input("dragon/roll/angle",roll_angle);
             register_input("dragon/pitch/angle",pitch_angle);

             register_input("/example/left2006/velocity", left2006_current_velocity);
             register_input("/target/pitch/aim/velocity",target_pitch_aim_velocity);
             register_input("/target/pitch/left/basic/torque",target_pitch_left_basic_torque);
    }

    void update() override {
        using namespace rmcs_msgs;
        if(count%100==0){
    
        RCLCPP_INFO(get_logger(), "pitch_measure_angle is:%lf",*pitch_angle);
        RCLCPP_INFO(get_logger(), "roll_measure_angle is:%lf",*roll_angle);

        }
        count++;
    }


private:
    rclcpp::Logger logger_;

    InputInterface<double> pitch_angle;
    InputInterface<double> roll_angle;
    InputInterface<double> left2006_current_velocity;
    InputInterface<double> target_pitch_aim_velocity;
    InputInterface<double> target_pitch_left_basic_torque;

    int count = 0;
};

} // namespace rmcs_core::example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::example::info, rmcs_executor::Component)