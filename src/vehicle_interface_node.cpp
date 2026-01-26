#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/global_position_target.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class VehicleInterfaceNode : public rclcpp::Node
{
public:
  VehicleInterfaceNode()
    : Node("vehicle_interface_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("mavros_namespace", "/mavros");
    this->declare_parameter<std::string>("fcu_url", "udp://:14540@127.0.0.1:14557");
    this->declare_parameter<double>("connection_timeout", 10.0);
    
    mavros_namespace_ = this->get_parameter("mavros_namespace").as_string();
    fcu_url_ = this->get_parameter("fcu_url").as_string();
    
    // Initialize TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Subscribers - MAVROS state and data
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      mavros_namespace_ + "/state", 10,
      std::bind(&VehicleInterfaceNode::state_callback, this, std::placeholders::_1));
    
    local_pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      mavros_namespace_ + "/local_position/pose", 10,
      std::bind(&VehicleInterfaceNode::local_position_callback, this, std::placeholders::_1));
    
    global_pos_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      mavros_namespace_ + "/global_position/global", 10,
      std::bind(&VehicleInterfaceNode::global_position_callback, this, std::placeholders::_1));
    
    // Publishers - Commands to vehicle
    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      mavros_namespace_ + "/setpoint_position/local", 10);
    
    local_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      mavros_namespace_ + "/setpoint_velocity/cmd_vel", 10);
    
    position_target_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
      mavros_namespace_ + "/setpoint_raw/local", 10);
    
    // Service clients
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
      mavros_namespace_ + "/cmd/arming");
    
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
      mavros_namespace_ + "/set_mode");
    
    // Publishers for processed data
    vehicle_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "vehicle/odometry", 10);
    
    vehicle_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "vehicle/connected", 10);
    
    // Timer for periodic tasks
    timer_ = this->create_wall_timer(
      100ms, std::bind(&VehicleInterfaceNode::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Vehicle Interface Node initialized");
    RCLCPP_INFO(this->get_logger(), "MAVROS namespace: %s", mavros_namespace_.c_str());
    RCLCPP_INFO(this->get_logger(), "FCU URL: %s", fcu_url_.c_str());
  }

private:
  void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
    
    // Publish connection status
    std_msgs::msg::Bool connected_msg;
    connected_msg.data = current_state_.connected;
    vehicle_state_pub_->publish(connected_msg);
    
    if (current_state_.connected && !prev_connected_) {
      RCLCPP_INFO(this->get_logger(), "Connected to FCU");
    } else if (!current_state_.connected && prev_connected_) {
      RCLCPP_WARN(this->get_logger(), "Disconnected from FCU");
    }
    
    prev_connected_ = current_state_.connected;
  }
  
  void local_position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_local_position_ = *msg;
    
    // Publish as odometry
    nav_msgs::msg::Odometry odom;
    odom.header = msg->header;
    odom.pose.pose = msg->pose;
    vehicle_odom_pub_->publish(odom);
  }
  
  void global_position_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    current_global_position_ = *msg;
  }
  
  void timer_callback()
  {
    // Periodic status logging
    static auto last_log_time = this->now();
    if ((this->now() - last_log_time).seconds() > 5.0) {
      if (current_state_.connected) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "FCU: Connected | Armed: %s | Mode: %s",
          current_state_.armed ? "YES" : "NO",
          current_state_.mode.c_str());
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "FCU: Not connected");
      }
      last_log_time = this->now();
    }
  }
  
  // Helper functions for vehicle control
  bool arm_vehicle(bool arm)
  {
    if (!arming_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "Arming service not available");
      return false;
    }
    
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    
    auto result = arming_client_->async_send_request(request);
    // Note: In production, you'd want to wait for the result properly
    RCLCPP_INFO(this->get_logger(), "Sending %s command", arm ? "ARM" : "DISARM");
    return true;
  }
  
  bool set_mode(const std::string& mode)
  {
    if (!set_mode_client_->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "Set mode service not available");
      return false;
    }
    
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    
    auto result = set_mode_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Setting mode to: %s", mode.c_str());
    return true;
  }
  
  void publish_position_setpoint(double x, double y, double z, double yaw = 0.0)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    
    // Convert yaw to quaternion (simplified)
    pose.pose.orientation.w = cos(yaw / 2.0);
    pose.pose.orientation.z = sin(yaw / 2.0);
    
    local_pos_pub_->publish(pose);
  }
  
  void publish_velocity_setpoint(double vx, double vy, double vz, double vyaw = 0.0)
  {
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = this->now();
    twist.header.frame_id = "base_link";
    twist.twist.linear.x = vx;
    twist.twist.linear.y = vy;
    twist.twist.linear.z = vz;
    twist.twist.angular.z = vyaw;
    
    local_vel_pub_->publish(twist);
  }
  
  // Member variables
  std::string mavros_namespace_;
  std::string fcu_url_;
  
  mavros_msgs::msg::State current_state_;
  geometry_msgs::msg::PoseStamped current_local_position_;
  sensor_msgs::msg::NavSatFix current_global_position_;
  
  bool prev_connected_ = false;
  
  // ROS2 objects
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr global_pos_sub_;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr local_vel_pub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr position_target_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vehicle_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vehicle_state_pub_;
  
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}

