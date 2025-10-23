#include <cmath>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::signal_processor {

class SineCosineGenerator
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SineCosineGenerator()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        
        // 从参数服务器读取角频率ω
        omega_ = get_parameter("omega").as_double();
        
        // 注册两个输出接口,One input
        register_output("/signal/sine", sine_output_,0.0);
        register_output("/signal/cosine", cosine_output_,0.0);
        register_input("/predefined/update_rate", update_rate_); 
        // 初始化时间和计数器
        time_ = 0.0;
        update_count_ = 0;
    }

    void update() override {
        // 计算当前时间的正弦和余弦值
        double sine_value = std::sin(omega_ * time_);
        double cosine_value = std::cos(omega_ * time_);
        
        // 输出到接口
        *sine_output_ = sine_value;
        *cosine_output_ = cosine_value;
        
        // 每100次更新打印一次数据（10Hz），避免打印过于频繁
        if (update_count_ % 100 == 0) {
            RCLCPP_INFO(get_logger(), "Sine: %.3f, Cosine: %.3f", sine_value, cosine_value);
        }
        
        // 更新时间，1000Hz对应时间步长为0.001秒
        time_ += 1.0 / *update_rate_;
        update_count_++;
        
        // 防止时间无限增长，每1000秒重置一次（可选）
        if (time_ > 1000.0) {
            time_ = 0.0;
        }
    }

private:
    double omega_;          // 角频率ω
    double time_;           // 当前时间
    int update_count_;      // 更新计数器
    InputInterface<double> update_rate_;
    OutputInterface<double> sine_output_;    // 正弦输出接口
    OutputInterface<double> cosine_output_;  // 余弦输出接口
};

} // namespace rmcs_core::controller::signal_processor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::signal_processor::SineCosineGenerator, rmcs_executor::Component)