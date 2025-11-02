// 外设调用：以dr16为例
// 驱动电机：以GM6020为例

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "librmcs/client/cboard.hpp"
#include <cmath>
#include <cstdint>
#include <rcl/publisher.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include "hardware/device/bmi088.hpp"
#include <rmcs_description/tf_description.hpp>
#include "librmcs/device/bmi088.hpp"
#include <math.h>
#define fc 10.0 //Corner Frequency
#define fs 1000.0 //Sampling Frequency
namespace rmcs_core::hardware {

class DeviceExample
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::client::CBoard {

public:
    DeviceExample()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , librmcs::client::CBoard{static_cast<int>(get_parameter("usb_pid").as_int())}
        , logger_(get_logger())
        , CBoard_command_(create_partner_component<CBoardCommand>(get_component_name() + "_command", *this))
        , dr16_(*this)
        , M2006_(*this, *CBoard_command_, "/example/M2006")
        ,left2006_(*this,*CBoard_command_,"/example/left2006")
        ,right2006_(*this,*CBoard_command_,"/example/right2006")
        ,bmi088_(1000,0.2,0.0)
        , transmit_buffer_(*this, 32)
        , event_thread_([this]() { handle_events(); }) {

     // M2006_.configure(device::DjiMotor::Config{device::DjiMotor::Type::M2006});
        left2006_.configure(device::DjiMotor::Config{device::DjiMotor::Type::M2006});
        right2006_.configure(device::DjiMotor::Config{device::DjiMotor::Type::M2006});

       
        //register_output("/dragon/roll/angle", roll_angle);
        register_output("/dragon/roll/angle_g", roll_angle_g_);
        register_output("/dragon/roll/angle_a", roll_angle_a_);
        register_output("/dragon/pitch/angle_g", pitch_angle_g_);
        register_output("/dragon/pitch/angle_a", pitch_angle_a_);
       // register_output("/tf", tf_);
       // register_input("/dragon/angle/torque", angle_torque);
      //  register_input("/example/left2006/angle", left2006_current_angle);
      //  register_input("/example/right2006/angle", right2006_current_angle);
      //  bmi088_.set_coordinate_mapping([](double x, double y, double z) {
            // Get the mapping with the following code.
            // The rotation angle must be an exact multiple of 90 degrees, otherwise use a matrix.

            // Eigen::AngleAxisd pitch_link_to_imu_link{
            //     std::numbers::pi / 2, Eigen::Vector3d::UnitZ()};
            // Eigen::Vector3d mapping = pitch_link_to_imu_link * Eigen::Vector3d{1, 2, 3};
            // std::cout << mapping << std::endl;*right_motor_aim_velocity_

           // return std::make_tuple(-y, x, z);
      //  });

    }

    ~DeviceExample() override {
        stop_handling_events();
        event_thread_.join();
    }

    void update() override {
        dr16_.update_status();
        update_motors();
        update_imu();
        if(count%200==0)
        {
          //  RCLCPP_INFO(get_logger(), "%lf", *dragon_yaw_velocity_imu_);
          //  RCLCPP_INFO(get_logger(), "%lf",*dragon_pitch_velocity_imu_);
          //  RCLCPP_INFO(get_logger(), "088_original_data:%lf",*dragon_roll_velocity_imu_); //
           // RCLCPP_INFO(get_logger(), "088_roll_angle:%lf",*roll_angle);
           // RCLCPP_INFO(get_logger(), "088_origin_angle_a_data:%lf",*roll_angle_a_);
          //  RCLCPP_INFO(get_logger(), "088_origin_angle_g_data:%lf",*roll_angle_g_);
          //  RCLCPP_INFO(get_logger(), "088_calculated_angle_data:%lf",*roll_angle);
        }
        count++;
    }

    void command_update() {
        
        uint16_t can_commands[4];
// can_commands[0] = M2006_.generate_command();
        can_commands[0] = left2006_.generate_command();
        can_commands[1] = right2006_.generate_command();
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can1_transmission(0x1FF, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x1FE, std::bit_cast<uint64_t>(can_commands));

        can_commands[0] = 0;
        can_commands[1] = 0;
        can_commands[2] = 0;
        can_commands[3] = 0;
        transmit_buffer_.add_can2_transmission(0x200, std::bit_cast<uint64_t>(can_commands));

        transmit_buffer_.trigger_transmission();
    }

private:
    void update_motors() 
    {
      // M2006_.update_status(); 
    left2006_.update_status();
    right2006_.update_status();
    
    }

    void update_imu() {
        bmi088_.update_status();
        //以下是roll角
        Eigen::Quaterniond gimbal_imu_pose{bmi088_.q0(), bmi088_.q1(), bmi088_.q2(), bmi088_.q3()};
        /* tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
            gimbal_imu_pose.conjugate()); */

       
       //get tuoluoyi
       double dragon_roll_velocity_imu_ = bmi088_.gx() / std::numbers::pi * 180.0;
       *roll_angle_g_ += (dragon_roll_velocity_imu_) * dt;
        
        // get jiasudu
        double dragon_ax = bmi088_.ax();
        double dragon_ay = bmi088_.ay();
        double dragon_az = bmi088_.az();
        
        double jump180_angle= atan2(dragon_ax,dragon_az) / std::numbers::pi * 180.0;
        *roll_angle_a_ = convert_to_180_offset(jump180_angle);

        //get finall data
       //*roll_angle = alpha*(*roll_angle_g_)+(1.0-alpha)*(*roll_angle_a_);

       //以下是pitch角
       double dragon_pitch_velocity_imu_ = bmi088_.gx() / std::numbers::pi * 180.0;
       *pitch_angle_g_ += (dragon_roll_velocity_imu_) * dt;
        //double jump180_angle= atan2(dragon_ax,dragon_az) / std::numbers::pi * 180.0;
        //*pitch_angle_a_ = convert_to_180_offset(jump180_angle);
        *pitch_angle_a_=(atan2(dragon_ay,dragon_az) / std::numbers::pi * 180.0)+180;

     }
    
     double convert_to_180_offset(double current_angle) {
        double offset_from_180;
        if(current_angle>0)
        {
            offset_from_180=180-current_angle;
        }
        if(current_angle<0)
        {
            offset_from_180=180+current_angle;
        }
        return offset_from_180;
    }


    

protected:
    void can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
        uint8_t can_data_length) override {
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;

        if (can_id == 0x201) {
         // M2006_.store_status(can_data);
         left2006_.store_status(can_data);
        }

        if (can_id == 0x202) {
        right2006_.store_status(can_data);
        }
    }

    // void can2_receive_callback(
    //     uint32_t can_id, uint64_t can_data, bool is_extended_can_id, bool is_remote_transmission,
    //     uint8_t can_data_length) override;

    // void uart1_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override;

    // void uart2_receive_callback(const std::byte* data, uint8_t length) override;

    void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
        dr16_.store_status(uart_data, uart_data_length);
    }

  void accelerometer_receive_callback(int16_t x, int16_t y, int16_t z) override {
        bmi088_.store_accelerometer_status(x, y, z);
    }

    void gyroscope_receive_callback(int16_t x, int16_t y, int16_t z) override {
        bmi088_.store_gyroscope_status(x, y, z);
    }

private:
    rclcpp::Logger logger_;

    class CBoardCommand : public rmcs_executor::Component {
    public:
        explicit CBoardCommand(DeviceExample& cboard)
            : cboard_(cboard) {}

        void update() override { cboard_.command_update(); }

        DeviceExample& cboard_;
    };
    std::shared_ptr<CBoardCommand> CBoard_command_;

    // device
    device::Dr16 dr16_;

    device::DjiMotor M2006_;
    device::DjiMotor left2006_;
    device::DjiMotor right2006_;
    
    device::Bmi088 bmi088_;
  //  OutputInterface<rmcs_description::Tf> tf_;

 
    //OutputInterface<double> roll_angle;
    OutputInterface<double> roll_angle_a_;
    OutputInterface<double> roll_angle_g_;
    OutputInterface<double> pitch_angle_a_;
    OutputInterface<double> pitch_angle_g_;
    InputInterface<double> left2006_current_angle;
    InputInterface<double> right2006_current_angle;

    librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
    std::thread event_thread_;
   double prev_roll_angle_a_ = 0.0;
    int unwrap_count_ = 0;
   double alpha=1.0/(1+(fc/fs)) ;
    double prev_angle;
    double dt = 0.001;
    int count=0.0;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DeviceExample, rmcs_executor::Component)