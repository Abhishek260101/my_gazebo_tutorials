/**
 * @file walker_node.cpp
 * @author Abhishek Avhad
 * @brief Implementation of the WalkerNode class
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

#include "walker/walker_node.hpp"

WalkerNode::WalkerNode()
    : Node("walker_node"),
      rotation_direction_(1.0) {  // Start with clockwise rotation

  // Set up QoS profile for laser scan data
  // Using BestEffort reliability for better handling of high-frequency sensor
  // data
  auto qos_profile = rclcpp::QoS(10)
                         .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                         .history(rclcpp::HistoryPolicy::KeepLast);

  // Initialize publisher and subscriber
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", qos_profile,
      std::bind(&WalkerNode::laser_callback, this, std::placeholders::_1));

  // Start in forward state
  current_state_ = std::make_unique<ForwardState>(this);
  RCLCPP_INFO(this->get_logger(), "Walker node initialized in Forward state");
}

void WalkerNode::publish_velocity(double linear, double angular) {
  // Create and fill velocity message
  auto msg = geometry_msgs::msg::Twist();
  msg.linear.x = linear;    // Set forward/backward velocity
  msg.angular.z = angular;  // Set rotational velocity
  publisher_->publish(msg);
}

bool WalkerNode::is_path_clear(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) const {
  // Check front 90-degree arc (±45 degrees)
  // Implementation assumes 360 scan points (0-359)
  const int left_start = 0;     // Front center
  const int left_end = 17;      // 45 degrees to the left
  const int right_start = 343;  // 45 degrees to the right
  const int right_end = 359;    // Back to front center

  // Check left side of front arc
  for (int i = left_start; i <= left_end; ++i) {
    if (scan->ranges[i] < SAFE_DISTANCE) {
      RCLCPP_INFO(this->get_logger(),
                  "Obstacle detected in left arc at %.2f meters",
                  scan->ranges[i]);
      return false;
    }
  }

  // Check right side of front arc
  for (int i = right_start; i <= right_end; ++i) {
    if (scan->ranges[i] < SAFE_DISTANCE) {
      RCLCPP_INFO(this->get_logger(),
                  "Obstacle detected in right arc at %.2f meters",
                  scan->ranges[i]);
      return false;
    }
  }
  return true;  // No obstacles detected
}

void WalkerNode::change_state(std::unique_ptr<WalkerState> new_state) {
  // Log state transition
  RCLCPP_INFO(this->get_logger(), "Changing state from %s to %s",
              current_state_->get_state_name().c_str(),
              new_state->get_state_name().c_str());

  current_state_ = std::move(new_state);
}

void WalkerNode::laser_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Delegate scan handling to current state
  current_state_->handle(this, msg);
}
