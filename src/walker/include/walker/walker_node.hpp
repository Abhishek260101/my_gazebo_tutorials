#ifndef WALKER_NODE_HPP_
#define WALKER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include "walker/walker_state.hpp"

/**
 * @class WalkerNode
 * @brief Main ROS2 node class that implements the walker behavior
 *
 * This class implements a ROS2 node that makes a robot move forward until it 
 * encounters an obstacle, then rotates until the path is clear. It uses the
 * State pattern to manage different behaviors.
 */
class WalkerNode : public rclcpp::Node {
 public:
    /**
     * @brief Default constructor
     */
    WalkerNode();
    
    /**
     * @brief Constructor with parameters
     * @param options ROS2 node options
     * @param linear_vel Forward velocity in m/s
     * @param angular_vel Rotation velocity in rad/s
     * @param min_dist Minimum obstacle distance in meters
     */
    explicit WalkerNode(
        const rclcpp::NodeOptions& options,
        double linear_vel = 0.2,
        double angular_vel = 0.5,
        double warn_distance = 1.0,    // New: Warning threshold
        double crit_distance = 0.5,    // New: Critical threshold
        double emerg_distance = 0.3    // New: Emergency threshold
    );
    
    /**
     * @brief Changes the current state of the walker
     * @param new_state Unique pointer to the new state object
     */
    void change_state(std::unique_ptr<WalkerState> new_state);
    
    // Getter methods for parameters
    /**
     * @brief Gets the current linear velocity
     * @return Linear velocity in m/s
     */
    double get_linear_velocity() const { return linear_velocity_; }
    
    /**
     * @brief Gets the current angular velocity
     * @return Angular velocity in rad/s
     */
    double get_angular_velocity() const { return angular_velocity_; }
    
    /**
     * @brief Gets the minimum obstacle distance
     * @return Minimum distance in meters
     */
    double get_min_distance() const { return min_distance_; }
    
    /**
     * @brief Gets the name of the current state
     * @return String representing current state
     */
    std::string get_current_state_name() const { 
        return current_state_->get_state_name(); 
    }

    /**
     * @brief Callback function for laser scan messages
     * @param msg Shared pointer to laser scan message
     */
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    double get_warning_distance() const { return warning_distance_; }
    double get_critical_distance() const { return critical_distance_; }
    double get_emergency_distance() const { return emergency_distance_; }
    double get_current_linear_vel() const { return current_linear_vel_; }

    // Methods for testing purposes
    /**
     * @brief Gets the last published velocity command
     * @return Last velocity command sent to the robot
     */
    geometry_msgs::msg::Twist get_last_velocity_command() const {
        return last_cmd_;
    }
    
    /**
     * @brief Publishes a velocity command
     * @param cmd Velocity command to publish
     */
    void publish_velocity(const geometry_msgs::msg::Twist& cmd) {
        last_cmd_ = cmd;
        publisher_->publish(cmd);
    }

 private:
    // ROS2 communication members
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;  ///< Publisher for velocity commands
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;  ///< Subscriber for laser scan data
    
    // State machine members
    std::unique_ptr<WalkerState> current_state_;  ///< Current state of the walker
    bool rotate_clockwise_;  ///< Flag for rotation direction
    
    // Robot parameters
    double linear_velocity_;   ///< Forward speed in m/s
    double angular_velocity_;  ///< Rotation speed in rad/s
    double min_distance_;      ///< Minimum obstacle distance in m
    
    // Testing support
    geometry_msgs::msg::Twist last_cmd_;  ///< Stores last velocity command
    double warning_distance_;   // Distance to start slowing down
    double critical_distance_;  // Distance to stop and rotate
    double emergency_distance_; // Distance for emergency stop
    double current_linear_vel_; // Current linear velocity
};

#endif  // WALKER_NODE_HPP_