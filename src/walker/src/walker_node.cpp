#include "walker/walker_node.hpp"

WalkerNode::WalkerNode() 
    : Node("walker_node"),                // Initialize the ROS2 node
      rotate_clockwise_(true),            // Start with clockwise rotation
      linear_velocity_(0.2),              // Default forward speed
      angular_velocity_(0.5),             // Default rotation speed
      min_distance_(0.5),                 // Default minimum obstacle distance
      last_cmd_()                         // Initialize last command to zeros
{
    // Initialize ROS2 communications
    // Create publisher for robot velocity commands
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Create subscriber for laser scan data
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&WalkerNode::laser_callback, this, std::placeholders::_1));

    // Initialize state machine with forward state
    current_state_ = std::make_unique<ForwardState>(this);
    
    // Log initialization
    RCLCPP_INFO(this->get_logger(), 
        "Walker node initialized in %s state", 
        current_state_->get_state_name().c_str());
}

WalkerNode::WalkerNode(
    const rclcpp::NodeOptions& options,
    double linear_vel,
    double angular_vel,
    double min_dist)
    : Node("walker_node", options),
      rotate_clockwise_(true),
      linear_velocity_(linear_vel),
      angular_velocity_(angular_vel),
      min_distance_(min_dist),
      last_cmd_()
{
    // Same initialization as default constructor
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
    // Log state transition
    RCLCPP_INFO(this->get_logger(), 
        "State changing from %s to %s", 
        current_state_->get_state_name().c_str(),
        new_state->get_state_name().c_str());
    
    // Update current state
    current_state_ = std::move(new_state);
}

void WalkerNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Let the current state process the scan data
    auto velocity = current_state_->process_scan(msg);
    
    // Store and publish the velocity command
    last_cmd_ = velocity;
    publisher_->publish(velocity);
}