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
#define window_size_ 8
#define CALIBRATION_SAMPLES 1000
namespace rmcs_core::filter {
class medianfilter
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    medianfilter()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()),drift_slope_k(0.0), is_calibrated(false) {
       
  
        register_input("/dragon/roll/angle_g",roll_angle_g_);
        register_input("/dragon/roll/angle_a",roll_angle_a_);
        register_output("/dragon/roll/angle", roll_angle);
    }

    void update() override {
        using namespace rmcs_msgs;
        roll_angle_calculate();
        
        }
    
    void roll_angle_calculate()
    {
        //roll_filted_angle_g_= filter(*roll_angle_g_);
        //roll_filted_angle_a_= compensate(*roll_angle_a_);
        *roll_angle = alpha*(*roll_angle_a_)+(1.0-alpha)*(*roll_angle_g_);
    }

// 以下为处理加速度计的噪声滤波
    double filter(double new_value) {
        buffer_.push_back(new_value);
    
         if (buffer_.size() > window_size_) {
        buffer_.pop_front();
        }
    
        // 如果缓冲区还没满，直接返回当前值
        if (buffer_.size() < window_size_) {
        return new_value;
        }
    
        // 复制数据到排序缓冲区
        sorted_buffer_.assign(buffer_.begin(), buffer_.end());
        std::sort(sorted_buffer_.begin(), sorted_buffer_.end());
    
        // 返回中值
        return sorted_buffer_[window_size_ / 2];
         }

    /**
     * 校准函数：计算零漂函数
     * @param samples 采样数据向量（陀螺仪读数）
     * @param sample_interval 采样间隔（秒）
     * @return 计算出的零漂斜率 k&b
     */
    void calibrate() {
        
        if(samples.size() < 500) {
            return ;
        }
        if(samples.size()<2000){
        // 使用线性回归计算斜率
        int n = samples.size();
        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
        
        for (int i = 0; i < n; ++i) {
            double x = i * sample_interval;  // 时间
            double y = samples[i];           // 陀螺仪读数
            
            sum_x += x;
            sum_y += y;
            sum_xy += x * y;
            sum_xx += x * x;
        }
        
        // 计算斜率 k = (nΣxy - ΣxΣy) / (nΣx² - (Σx)²)
        drift_slope_k = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        drift_intercept_b = (sum_y - drift_slope_k * sum_x) / n;
        // 记录补偿开始时间
        sample_start_time = Clock::now();
        is_calibrated = true;
        }
    }
    
    /**
     * 应用零漂补偿
     * @param raw_gyro 原始陀螺仪读数
     * @return 补偿后的读数
     */
    double compensate(double raw_gyro) {
        if (!is_calibrated) {
            return raw_gyro;
        }
        
        // 计算从校准开始经过的时间（秒）
        auto current_time = Clock::now();
        double elapsed_time = std::chrono::duration<double>(current_time - sample_start_time).count();
                // 补偿公式：读数 - (k*t + b)
        double drift_compensation = drift_slope_k * elapsed_time + drift_intercept_b;
        return raw_gyro - drift_compensation;
    }
    
    /**
     * 重置补偿器（开始新的校准）
     */
    void reset() {
        drift_slope_k = 0.0;
        is_calibrated = false;
    }
    
   // bool calibrated() const { return is_calibrated; }
   // double getSlope() const { return drift_slope_k; }
private:
    using Clock = std::chrono::steady_clock;
    rclcpp::Logger logger_;
    double drift_slope_k;  // 零漂斜率
    bool is_calibrated;
    double drift_intercept_b;  // 零漂截距 b
    std::deque<double> buffer_;
    std::vector<double> sorted_buffer_;
    std::deque<double> samples;
    Clock::time_point sample_start_time;  // 补偿开始时间
    //Clock::time_point system_start_time;  // 系统开始时间

    InputInterface<double> roll_angle_g_;
    InputInterface<double> roll_angle_a_;
    OutputInterface<double> roll_angle;

    double roll_filted_angle_g_;
    double roll_filted_angle_a_;
    double alpha=1.0/(1+(fc/fs)) ;
    
    
};



} // namespace rmcs_core::example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::filter::medianfilter, rmcs_executor::Component)