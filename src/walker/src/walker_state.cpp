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
    
    // Get the front arc of ranges (-30 to +30 degrees)
    size_t start_idx = scan->ranges.size() / 6;  // -30 degrees
    size_t end_idx = scan->ranges.size() / 3;    // +30 degrees
    
    // Find minimum distance in the front arc
    float min_range = std::numeric_limits<float>::infinity();
    for (size_t i = start_idx; i < end_idx; ++i) {
        if (!std::isinf(scan->ranges[i]) && scan->ranges[i] < min_range) {
            min_range = scan->ranges[i];
        }
    }

    // Get thresholds
    double warn_dist = context_->get_warning_distance();
    double crit_dist = context_->get_critical_distance();
    double emerg_dist = context_->get_emergency_distance();
    double max_linear_vel = context_->get_linear_velocity();

    if (min_range <= emerg_dist) {
        // Emergency stop
        velocity.linear.x = 0.0;
        velocity.angular.z = 0.0;
        RCLCPP_WARN(rclcpp::get_logger("walker_node"), 
            "Emergency stop! Obstacle too close: %.2f m", min_range);
    }
    else if (min_range <= crit_dist) {
        // Critical distance - stop and rotate
        velocity.linear.x = 0.0;
        velocity.angular.z = 0.0;
        
        // Change to rotating state
        context_->change_state(
            std::make_unique<RotatingState>(
                context_, 
                static_cast<bool>(rand() % 2)
            )
        );
    }
    else if (min_range <= warn_dist) {
        // Warning distance - slow down proportionally
        double slow_factor = (min_range - crit_dist) / (warn_dist - crit_dist);
        velocity.linear.x = max_linear_vel * slow_factor;
        velocity.angular.z = 0.0;
        
        RCLCPP_INFO(rclcpp::get_logger("walker_node"), 
            "Slowing down. Distance: %.2f m, Speed: %.2f m/s", 
            min_range, velocity.linear.x);
    }
    else {
        // Clear path - full speed ahead
        velocity.linear.x = max_linear_vel;
        velocity.angular.z = 0.0;
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
    
    // Get the front arc of ranges (-30 to +30 degrees)
    size_t start_idx = scan->ranges.size() / 6;
    size_t end_idx = scan->ranges.size() / 3;
    
    // Find minimum distance in the front arc
    float min_range = std::numeric_limits<float>::infinity();
    for (size_t i = start_idx; i < end_idx; ++i) {
        if (!std::isinf(scan->ranges[i]) && scan->ranges[i] < min_range) {
            min_range = scan->ranges[i];
        }
    }
    
    // Only transition to forward if we have good clearance
    if (min_range > context_->get_warning_distance()) {
        // Path is clear with good margin - switch to forward
        context_->change_state(std::make_unique<ForwardState>(context_));
        velocity.linear.x = 0.0;
        velocity.angular.z = 0.0;
    } 
    else if (min_range <= context_->get_emergency_distance()) {
        // Emergency - stop rotation
        velocity.linear.x = 0.0;
        velocity.angular.z = 0.0;
        RCLCPP_WARN(rclcpp::get_logger("walker_node"), 
            "Emergency while rotating! Obstacle too close: %.2f m", min_range);
    }
    else {
        // Continue rotating
        velocity.linear.x = 0.0;
        velocity.angular.z = rotate_clockwise_ ? 
            -context_->get_angular_velocity() : 
            context_->get_angular_velocity();
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