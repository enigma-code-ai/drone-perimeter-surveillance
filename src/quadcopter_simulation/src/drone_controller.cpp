#include <memory>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class DroneController : public rclcpp::Node
{
public:
    DroneController() : Node("drone_controller")
    {
        // Parameters
        this->declare_parameter("mass", 1.5);
        this->declare_parameter("arm_length", 0.22);
        this->declare_parameter("thrust_coefficient", 2.95e-05);
        this->declare_parameter("max_thrust", 15.0);
        
        mass_ = this->get_parameter("mass").as_double();
        arm_length_ = this->get_parameter("arm_length").as_double();
        thrust_coeff_ = this->get_parameter("thrust_coefficient").as_double();
        max_thrust_ = this->get_parameter("max_thrust").as_double();
        
        // Initialize PID controllers
        initializePIDControllers();
        
        // Publishers
        motor_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/motor_commands", 10);
        
        // Subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&DroneController::cmdVelCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/drone/imu/data", 10,
            std::bind(&DroneController::imuCallback, this, std::placeholders::_1));
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/drone/pose", 10,
            std::bind(&DroneController::poseCallback, this, std::placeholders::_1));
        
        // Control timer (100 Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DroneController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Drone Controller initialized");
    }

private:
    // PID Controller structure
    struct PIDController {
        double kp, ki, kd;
        double error_sum = 0.0;
        double last_error = 0.0;
        double max_output, min_output;
        
        double compute(double error, double dt) {
            error_sum += error * dt;
            double d_error = (error - last_error) / dt;
            last_error = error;
            
            double output = kp * error + ki * error_sum + kd * d_error;
            
            // Clamp output
            if (output > max_output) output = max_output;
            if (output < min_output) output = min_output;
            
            return output;
        }
        
        void reset() {
            error_sum = 0.0;
            last_error = 0.0;
        }
    };
    
    void initializePIDControllers() {
        // Position controllers
        pos_x_pid_.kp = 1.5; pos_x_pid_.ki = 0.0; pos_x_pid_.kd = 0.5;
        pos_x_pid_.max_output = 10.0; pos_x_pid_.min_output = -10.0;
        
        pos_y_pid_.kp = 1.5; pos_y_pid_.ki = 0.0; pos_y_pid_.kd = 0.5;
        pos_y_pid_.max_output = 10.0; pos_y_pid_.min_output = -10.0;
        
        pos_z_pid_.kp = 2.0; pos_z_pid_.ki = 0.1; pos_z_pid_.kd = 1.0;
        pos_z_pid_.max_output = 20.0; pos_z_pid_.min_output = -20.0;
        
        // Attitude controllers
        roll_pid_.kp = 7.0; roll_pid_.ki = 0.0; roll_pid_.kd = 0.0;
        roll_pid_.max_output = 50.0; roll_pid_.min_output = -50.0;
        
        pitch_pid_.kp = 7.0; pitch_pid_.ki = 0.0; pitch_pid_.kd = 0.0;
        pitch_pid_.max_output = 50.0; pitch_pid_.min_output = -50.0;
        
        yaw_pid_.kp = 5.0; yaw_pid_.ki = 0.0; yaw_pid_.kd = 0.0;
        yaw_pid_.max_output = 30.0; yaw_pid_.min_output = -30.0;
        
        // Rate controllers
        roll_rate_pid_.kp = 0.15; roll_rate_pid_.ki = 0.05; roll_rate_pid_.kd = 0.003;
        roll_rate_pid_.max_output = 1.0; roll_rate_pid_.min_output = -1.0;
        
        pitch_rate_pid_.kp = 0.15; pitch_rate_pid_.ki = 0.05; pitch_rate_pid_.kd = 0.003;
        pitch_rate_pid_.max_output = 1.0; pitch_rate_pid_.min_output = -1.0;
        
        yaw_rate_pid_.kp = 0.2; yaw_rate_pid_.ki = 0.05; yaw_rate_pid_.kd = 0.0;
        yaw_rate_pid_.max_output = 1.0; yaw_rate_pid_.min_output = -1.0;
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_vel_x_ = msg->linear.x;
        target_vel_y_ = msg->linear.y;
        target_vel_z_ = msg->linear.z;
        target_yaw_rate_ = msg->angular.z;
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Extract orientation
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        
        tf2::Matrix3x3 m(q);
        m.getRPY(current_roll_, current_pitch_, current_yaw_);
        
        // Extract angular velocities
        roll_rate_ = msg->angular_velocity.x;
        pitch_rate_ = msg->angular_velocity.y;
        yaw_rate_ = msg->angular_velocity.z;
    }
    
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_x_ = msg->pose.position.x;
        current_y_ = msg->pose.position.y;
        current_z_ = msg->pose.position.z;
    }
    
    void controlLoop() {
        double dt = 0.01; // 100 Hz
        
        // Position control -> desired accelerations
        double acc_x = pos_x_pid_.compute(target_x_ - current_x_, dt);
        double acc_y = pos_y_pid_.compute(target_y_ - current_y_, dt);
        double acc_z = pos_z_pid_.compute(target_z_ - current_z_, dt) + 9.81; // gravity compensation
        
        // Convert desired accelerations to desired angles
        double target_roll = (acc_x * sin(current_yaw_) - acc_y * cos(current_yaw_)) / 9.81;
        double target_pitch = (acc_x * cos(current_yaw_) + acc_y * sin(current_yaw_)) / 9.81;
        
        // Limit angles
        target_roll = std::clamp(target_roll, -0.52, 0.52);  // ±30 degrees
        target_pitch = std::clamp(target_pitch, -0.52, 0.52);
        
        // Attitude control -> desired rates
        double target_roll_rate = roll_pid_.compute(target_roll - current_roll_, dt);
        double target_pitch_rate = pitch_pid_.compute(target_pitch - current_pitch_, dt);
        double target_yaw_rate = yaw_pid_.compute(target_yaw_ - current_yaw_, dt);
        
        // Rate control -> motor mixing
        double roll_torque = roll_rate_pid_.compute(target_roll_rate - roll_rate_, dt);
        double pitch_torque = pitch_rate_pid_.compute(target_pitch_rate - pitch_rate_, dt);
        double yaw_torque = yaw_rate_pid_.compute(target_yaw_rate - yaw_rate_, dt);
        
        // Total thrust
        double total_thrust = mass_ * acc_z;
        
        // Motor mixing for X configuration
        // Motor 0: Front, Motor 1: Right, Motor 2: Back, Motor 3: Left
        double motor_0 = total_thrust/4 - pitch_torque/2 + yaw_torque/4;
        double motor_1 = total_thrust/4 - roll_torque/2 - yaw_torque/4;
        double motor_2 = total_thrust/4 + pitch_torque/2 + yaw_torque/4;
        double motor_3 = total_thrust/4 + roll_torque/2 - yaw_torque/4;
        
        // Convert thrust to motor speed (simplified)
        motor_0 = sqrt(std::max(0.0, motor_0 / thrust_coeff_));
        motor_1 = sqrt(std::max(0.0, motor_1 / thrust_coeff_));
        motor_2 = sqrt(std::max(0.0, motor_2 / thrust_coeff_));
        motor_3 = sqrt(std::max(0.0, motor_3 / thrust_coeff_));
        
        // Publish motor commands
        auto motor_msg = std_msgs::msg::Float64MultiArray();
        motor_msg.data = {motor_0, motor_1, motor_2, motor_3};
        motor_command_pub_->publish(motor_msg);
    }
    
    // ROS2 interfaces
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_command_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // PID controllers
    PIDController pos_x_pid_, pos_y_pid_, pos_z_pid_;
    PIDController roll_pid_, pitch_pid_, yaw_pid_;
    PIDController roll_rate_pid_, pitch_rate_pid_, yaw_rate_pid_;
    
    // Drone state
    double current_x_ = 0.0, current_y_ = 0.0, current_z_ = 0.0;
    double current_roll_ = 0.0, current_pitch_ = 0.0, current_yaw_ = 0.0;
    double roll_rate_ = 0.0, pitch_rate_ = 0.0, yaw_rate_ = 0.0;
    
    // Target state
    double target_x_ = 0.0, target_y_ = 0.0, target_z_ = 1.0;
    double target_yaw_ = 0.0;
    double target_vel_x_ = 0.0, target_vel_y_ = 0.0, target_vel_z_ = 0.0;
    double target_yaw_rate_ = 0.0;
    
    // Physical parameters
    double mass_;
    double arm_length_;
    double thrust_coeff_;
    double max_thrust_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneController>());
    rclcpp::shutdown();
    return 0;
}