#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#define fc 10.0 //Corner Frequency
#define fs 1000.0 //Sampling Frequency

namespace rmcs_core::example {
class pitchcontroll
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    pitchcontroll()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
         {
       register_input("/dragon/pitch/angle_g",pitch_angle_g_);
        register_input("/dragon/pitch/angle_a",pitch_angle_a_);
        register_output("/dragon/pitch/angle", pitch_angle);
    }

    void update() override {
        using namespace rmcs_msgs;
        if(count%100==0){
        RCLCPP_INFO(get_logger(), "pitch_g_data:%lf",*pitch_angle_g_);
        RCLCPP_INFO(get_logger(), "pitch_a_data:%lf",*pitch_angle_a_);
        }
        count++;
    }
    void pitch_angle_calculate()
    {
        *pitch_angle = *pitch_angle_a_;
    }

private:
    rclcpp::Logger logger_;

    InputInterface<double> pitch_angle_g_; //21~35
    InputInterface<double> pitch_angle_a_;
    OutputInterface<double> pitch_angle;
    double alpha=1.0/(1+(fc/fs)) ;
    int count=0;
};

} // namespace rmcs_core::example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::example::pitchcontroll, rmcs_executor::Component)