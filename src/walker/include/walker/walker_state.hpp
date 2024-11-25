/**
 * @file walker_state.hpp
 * @author Abhishek Avhad
 * @brief Header file for the Walker State classes
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

#ifndef WALKER_STATE_HPP_
#define WALKER_STATE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class WalkerNode;

/**
 * @class WalkerState
 * @brief Abstract base class for walker states
 *
 * Defines the interface for different states in the walker's state machine.
 */
class WalkerState {
 public:
  virtual ~WalkerState() = default;

  /**
   * @brief Handles laser scan data according to current state
   * @param node Pointer to the walker node
   * @param scan Shared pointer to laser scan data
   */
  virtual void handle(WalkerNode* node,
                      const sensor_msgs::msg::LaserScan::SharedPtr scan) = 0;

  /**
   * @brief Gets the name of the current state
   * @return String representing the state name
   */
  virtual std::string get_state_name() const = 0;
};

/**
 * @class ForwardState
 * @brief State for moving forward
 *
 * Implements behavior for moving forward until an obstacle is detected.
 */
class ForwardState : public WalkerState {
 public:
  ForwardState() = default;
  explicit ForwardState(WalkerNode* node) {}

  void handle(WalkerNode* node,
              const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
  std::string get_state_name() const override { return "Forward"; }
};

/**
 * @class RotationState
 * @brief State for rotating to avoid obstacles
 *
 * Implements behavior for rotating until a clear path is found.
 */
class RotationState : public WalkerState {
 public:
  /**
   * @brief Default constructor
   */
  RotationState() : initial_rotation_(true), rotation_timer_(nullptr) {}

  /**
   * @brief Constructor with node parameter
   * @param node Pointer to walker node
   */
  explicit RotationState(WalkerNode* node)
      : initial_rotation_(true), rotation_timer_(nullptr) {}

  void handle(WalkerNode* node,
              const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
  std::string get_state_name() const override { return "Rotating"; }

 private:
  // Flag for initial rotation
  bool initial_rotation_;
  //< Timer for rotation duration
  rclcpp::TimerBase::SharedPtr rotation_timer_;
};

#endif  // WALKER_STATE_HPP_