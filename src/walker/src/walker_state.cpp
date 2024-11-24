#include "walker/walker_state.hpp"
#include "walker/walker_node.hpp"
#include <chrono>
#include <thread>

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
    
    if (min_range <= crit_dist + 0.3) {  // Add 0.3m buffer for safety
        // Stop
        velocity.linear.x = 0.0;
        velocity.angular.z = 0.0;
        
        // Log the detection
        RCLCPP_INFO(rclcpp::get_logger("walker_node"), 
            "Obstacle detected at %.2f meters. Stopping...", min_range);
        
        // Sleep for 1 second to ensure complete stop
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // Change to rotating state
        context_->change_state(
            std::make_unique<RotatingState>(
                context_, 
                static_cast<bool>(rand() % 2)
            )
        );
    }
    else if (min_range <= warn_dist) {
        // In warning zone - slow down proportionally
        double slow_factor = (min_range - crit_dist) / (warn_dist - crit_dist);
        velocity.linear.x = context_->get_linear_velocity() * slow_factor;
        velocity.angular.z = 0.0;
        
        RCLCPP_INFO(rclcpp::get_logger("walker_node"), 
            "Approaching obstacle at %.2f meters. Slowing down to %.2f m/s", 
            min_range, velocity.linear.x);
    }
    else {
        // Clear path - full speed
        velocity.linear.x = context_->get_linear_velocity();
        velocity.angular.z = 0.0;
    }
    
    return velocity;
}

geometry_msgs::msg::Twist RotatingState::process_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    
    geometry_msgs::msg::Twist velocity;
    
    // Extract the front arc of ranges (-30 to +30 degrees)
    size_t start_idx = scan->ranges.size() / 6;  // -30 degrees
    size_t end_idx = scan->ranges.size() / 3;    // +30 degrees
    
    // Find minimum distance in the front arc
    float min_range = std::numeric_limits<float>::infinity();
    for (size_t i = start_idx; i < end_idx; ++i) {
        if (!std::isinf(scan->ranges[i]) && scan->ranges[i] < min_range) {
            min_range = scan->ranges[i];
        }
    }

    // Only switch to forward if we have good clearance
    if (min_range > context_->get_warning_distance()) {
        // Path is clear with good margin
        RCLCPP_INFO(rclcpp::get_logger("walker_node"), 
            "Path clear at %.2f meters. Resuming forward movement.", min_range);
        
        // Sleep briefly before changing direction
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Change to forward state
        context_->change_state(std::make_unique<ForwardState>(context_));
        
        // Stop rotation before moving forward
        velocity.linear.x = 0.0;
        velocity.angular.z = 0.0;
    }
    else if (min_range <= context_->get_emergency_distance()) {
        // Emergency - stop rotation
        velocity.linear.x = 0.0;
        velocity.angular.z = 0.0;
        RCLCPP_WARN(rclcpp::get_logger("walker_node"), 
            "Emergency! Obstacle very close: %.2f meters", min_range);
    }
    else {
        // Continue rotating
        velocity.linear.x = 0.0;
        double rotation_speed = context_->get_angular_velocity();
        // Reduce rotation speed when closer to obstacles
        if (min_range < context_->get_critical_distance() * 1.5) {
            rotation_speed *= 0.5;  // Slow rotation when close to obstacles
        }
        velocity.angular.z = rotate_clockwise_ ? -rotation_speed : rotation_speed;
    }
    
    return velocity;
}