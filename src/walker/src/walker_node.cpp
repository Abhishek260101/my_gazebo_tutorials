#include "walker/walker_node.hpp"

WalkerNode::WalkerNode() 
    : Node("walker_node"),                // Initialize the ROS2 node
      rotate_clockwise_(true),            // Start with clockwise rotation
      linear_velocity_(0.15),              // Default forward speed
      angular_velocity_(0.3),             // Default rotation speed
      min_distance_(0.5),                 // Default minimum distance
      warning_distance_(1.0),             // Default warning distance
      critical_distance_(0.7),            // Default critical distance
      emergency_distance_(0.5),           // Default emergency distance
      current_linear_vel_(0.2)            // Start at full speed
{
    // Initialize ROS2 communications
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&WalkerNode::laser_callback, this, std::placeholders::_1));

    // Initialize state machine with forward state
    current_state_ = std::make_unique<ForwardState>(this);
    
    RCLCPP_INFO(this->get_logger(), 
        "Walker node initialized in %s state", 
        current_state_->get_state_name().c_str());
}

WalkerNode::WalkerNode(
    const rclcpp::NodeOptions& options,
    double linear_vel,
    double angular_vel,
    double warn_distance,
    double crit_distance,
    double emerg_distance)
    : Node("walker_node", options),
      rotate_clockwise_(true),
      linear_velocity_(linear_vel),
      angular_velocity_(angular_vel),
      min_distance_(crit_distance),        // Use critical distance as min_distance
      warning_distance_(warn_distance),
      critical_distance_(crit_distance),
      emergency_distance_(emerg_distance),
      current_linear_vel_(linear_vel)
{
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&WalkerNode::laser_callback, this, std::placeholders::_1));
    
    current_state_ = std::make_unique<ForwardState>(this);
    
    RCLCPP_INFO(this->get_logger(), 
        "Walker node initialized with custom parameters");
}

void WalkerNode::change_state(std::unique_ptr<WalkerState> new_state) {
    RCLCPP_INFO(this->get_logger(), 
        "State changing from %s to %s", 
        current_state_->get_state_name().c_str(),
        new_state->get_state_name().c_str());
    
    current_state_ = std::move(new_state);
}

void WalkerNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto velocity = current_state_->process_scan(msg);
    last_cmd_ = velocity;
    publisher_->publish(velocity);
}