#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <deque>
#include <chrono>
#include <vector>
#include <algorithm>
#define sample_interval 0.001
#define fc 10.0 //Corner Frequency
#define fs 1000.0 //Sampling Frequency

namespace rmcs_core::angle {
class anglecalculator
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    anglecalculator()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()){
       
  
        register_input("/dragon/roll/angle_g", roll_angle_g_);
        register_input("/dragon/roll/angle_a", roll_angle_a_);
        register_input("/dragon/pitch/angle_g", pitch_angle_g_);
        register_input("/dragon/pitch/angle_a", pitch_angle_a_);

        register_output("dragon/roll/angle",roll_angle);
        register_output("dragon/pitch/angle",pitch_angle);
    }

    void update() override {
        using namespace rmcs_msgs;
        angle_calculate();
        
        }
    
    void angle_calculate()
    {
        *roll_angle = alpha*(*roll_angle_a_)+(1.0-alpha)*(*roll_angle_g_);

        *pitch_angle = alpha*(*pitch_angle_a_)+(1.0-alpha)*(*pitch_angle_g_);
    }

private:
    using Clock = std::chrono::steady_clock;
    rclcpp::Logger logger_;


    InputInterface<double> roll_angle_g_;
    InputInterface<double> roll_angle_a_;
    InputInterface<double> pitch_angle_g_;
    InputInterface<double> pitch_angle_a_;

    OutputInterface<double> pitch_angle;
    OutputInterface<double> roll_angle;

    double alpha=1.0/(1+(fc/fs)) ;
    
    
};



} // namespace rmcs_core::example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::angle::anglecalculator, rmcs_executor::Component)