#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::signal_processor {

class SignalAdder
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SignalAdder()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        
        // 注册两个输入接口（接收组件A的输出）
        register_input("/signal/sine", sine_input_);
        register_input("/signal/cosine", cosine_input_);
        
        // 注册输出接口（两个输入的和）
        register_output("/signal/sum", sum_output_,0.0);
        
        // 初始化计数器
        update_count_ = 0;
    }

    void update() override {
        // 检查输入是否就绪
        if (!sine_input_.ready() || !cosine_input_.ready()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                                "Waiting for sine and cosine inputs...");
            return;
        }
        
        // 读取输入值
        double sine_value = *sine_input_;
        double cosine_value = *cosine_input_;
        
        // 计算和并输出
        double sum = sine_value + cosine_value;
        *sum_output_ = sum;
        
        // 每100次更新打印一次数据（10Hz），避免打印过于频繁
        if (update_count_ % 100 == 0) {
            RCLCPP_INFO(get_logger(), "Sine: %.3f + Cosine: %.3f = Sum: %.3f", 
                       sine_value, cosine_value, sum);
        }
        
        update_count_++;
    }

private:
    InputInterface<double> sine_input_;     // 正弦输入接口
    InputInterface<double> update_rate_;
    InputInterface<double> cosine_input_;   // 余弦输入接口
    OutputInterface<double> sum_output_;    // 和输出接口
    int update_count_;                      // 更新计数器
};

} // namespace rmcs_core::controller::signal_processor

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::signal_processor::SignalAdder, rmcs_executor::Component)