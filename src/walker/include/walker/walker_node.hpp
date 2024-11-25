/**
 * @file walker_node.hpp
 * @author Abhishek Avhad
 * @brief Header file for the WalkerNode class
 * @version 1.0
 * @date 2024-11-24
 *
 * @copyright Copyright (c) 2024
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef WALKER_NODE_HPP_
#define WALKER_NODE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "walker/walker_state.hpp"

/**
 * @class WalkerNode
 * @brief Main ROS2 node class implementing walker behavior using State pattern
 *
 * This class manages the robot's movement, obstacle detection, and state
 * transitions. It implements a simple walker algorithm that moves forward until
 * detecting an obstacle, then rotates to find a clear path.
 */
class WalkerNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for WalkerNode
   *
   * Initializes the ROS2 node, publishers, subscribers, and sets initial state
   */
  WalkerNode();

  /**
   * @brief Changes the current state of the walker
   * @param new_state Unique pointer to the new state object
   */
  void change_state(std::unique_ptr<WalkerState> new_state);

  /**
   * @brief Publishes velocity commands to the robot
   * @param linear Linear velocity in m/s
   * @param angular Angular velocity in rad/s
   */
  void publish_velocity(double linear, double angular);

  /**
   * @brief Checks if the path ahead is clear of obstacles
   * @param scan Shared pointer to laser scan data
   * @return true if path is clear, false if obstacle detected
   */
  bool is_path_clear(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;

  /**
   * @brief Gets the current rotation direction
   * @return Current rotation direction (1.0 or -1.0)
   */
  double get_rotation_direction() const { return rotation_direction_; }

  /**
   * @brief Toggles the rotation direction between clockwise and
   * counter-clockwise
   */
  void toggle_rotation_direction() { rotation_direction_ *= -1.0; }

  /**
   * @brief Creates a wall timer for state transitions
   * @param period Timer period
   * @param callback Timer callback function
   * @return Shared pointer to created timer
   */
  rclcpp::TimerBase::SharedPtr create_wall_timer(
      const std::chrono::duration<double>& period,
      std::function<void()> callback) {
    return Node::create_wall_timer(period, callback);
  }

 private:
  /**
   * @brief Callback function for laser scan messages
   * @param msg Shared pointer to laser scan message
   */
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // ROS2 communication members
  //< Velocity command publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  //< Laser scan subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

  std::unique_ptr<WalkerState> current_state_;  ///< Current state of the walker
  double rotation_direction_;  ///< Current rotation direction (1.0 or -1.0)

  // Constants for robot behavior
  // Minimum safe distance to obstacles (meters)
  const double SAFE_DISTANCE = 0.8;
  const double LINEAR_VELOCITY = 0.3;
  const double ANGULAR_VELOCITY = 0.3;
};

#endif  // WALKER_NODE_HPP
