/**
 * @file walker_state.cpp
 * @author Abhishek Avhad
 * @brief Implementation of the Walker State classes
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

#include "walker/walker_state.hpp"

#include "walker/walker_node.hpp"

void ForwardState::handle(WalkerNode* node,
                          const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Check if path is clear and move accordingly
  if (node->is_path_clear(scan)) {
    // No obstacles detected, continue forward
    node->publish_velocity(0.3, 0.0);
  } else {
    // Obstacle detected, prepare for rotation
    node->toggle_rotation_direction();  // Alternate rotation direction
    RCLCPP_INFO(
        node->get_logger(),
        "Obstacle detected, changing rotation direction and starting rotation");

    // Transition to rotation state
    node->change_state(std::make_unique<RotationState>(node));
  }
}

void RotationState::handle(WalkerNode* node,
                           const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (initial_rotation_) {
    // In initial rotation period
    // Rotate according to current direction
    node->publish_velocity(0.0, 0.3 * node->get_rotation_direction());

    // Set up timer for initial rotation if not already set
    if (!rotation_timer_) {
      // Create 10-second timer for initial rotation
      rotation_timer_ =
          node->create_wall_timer(std::chrono::seconds(10), [this]() {
            initial_rotation_ = false;
            rotation_timer_ = nullptr;
          });

      RCLCPP_INFO(node->get_logger(),
                  "Starting initial 10-second rotation period");
    }
  } else {
    // After initial rotation period
    if (node->is_path_clear(scan)) {
      // Found clear path, transition to forward movement
      RCLCPP_INFO(node->get_logger(), "Path is clear, moving forward");
      node->change_state(std::make_unique<ForwardState>(node));
    } else {
      // Continue rotating until clear path is found
      node->publish_velocity(0.0, 0.3 * node->get_rotation_direction());
      RCLCPP_INFO(node->get_logger(), "Path blocked, continuing rotation");
    }
  }
}
