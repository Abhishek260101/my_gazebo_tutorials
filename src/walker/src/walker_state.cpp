#include "walker/walker_state.hpp"
#include "walker/walker_node.hpp"

bool WalkerState::is_path_blocked(
    const std::vector<float>& ranges, 
    double min_distance) {
    // Check each range value in the provided array
    for (float range : ranges) {
        // If range is less than minimum distance and not infinite
        if (!std::isinf(range) && range < min_distance) {
            return true;  // Path is blocked
        }
    }
    return false;  // Path is clear
}

geometry_msgs::msg::Twist ForwardState::process_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    
    geometry_msgs::msg::Twist velocity;
    
    // Extract the front arc of ranges (-30 to +30 degrees)
    size_t start_idx = scan->ranges.size() / 6;  // -30 degrees
    size_t end_idx = scan->ranges.size() / 3;    // +30 degrees
    
    // Create vector of relevant ranges
    std::vector<float> front_ranges(
        scan->ranges.begin() + start_idx,
        scan->ranges.begin() + end_idx
    );
    
    // Check if path is blocked
    if (is_path_blocked(front_ranges, context_->get_min_distance())) {
        // Change to rotating state with random direction
        context_->change_state(
            std::make_unique<RotatingState>(
                context_, 
                static_cast<bool>(rand() % 2)  // Random rotation direction
            )
        );
        // Stop the robot before rotating
        velocity.linear.x = 0.0;
        velocity.angular.z = 0.0;
    } else {
        // Path is clear, create forward movement command
        velocity = create_forward_command(context_->get_linear_velocity());
    }
    
    return velocity;
}

geometry_msgs::msg::Twist ForwardState::create_forward_command(double linear_vel) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_vel;  // Set forward velocity
    cmd.angular.z = 0.0;        // No rotation
    return cmd;
}

geometry_msgs::msg::Twist RotatingState::process_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    
    geometry_msgs::msg::Twist velocity;
    
    // Extract the front arc of ranges (-30 to +30 degrees)
    size_t start_idx = scan->ranges.size() / 6;  // -30 degrees
    size_t end_idx = scan->ranges.size() / 3;    // +30 degrees
    
    // Create vector of relevant ranges
    std::vector<float> front_ranges(
        scan->ranges.begin() + start_idx,
        scan->ranges.begin() + end_idx
    );
    
    // Check if path is now clear
    if (!is_path_blocked(front_ranges, context_->get_min_distance())) {
        // Change to forward state
        context_->change_state(std::make_unique<ForwardState>(context_));
        // Stop rotation before moving forward
        velocity.linear.x = 0.0;
        velocity.angular.z = 0.0;
    } else {
        // Path still blocked, continue rotating
        velocity = create_rotation_command(
            context_->get_angular_velocity(),
            rotate_clockwise_
        );
    }
    
    return velocity;
}

geometry_msgs::msg::Twist RotatingState::create_rotation_command(
    double angular_vel, bool clockwise) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;  // No forward movement
    // Set rotation direction
    cmd.angular.z = clockwise ? -angular_vel : angular_vel;
    return cmd;
}