/**
 * @file main.cpp
 * @author Abhishek Avhad (avhad.abhishek005@gmail.com)
 * @brief Main entry point for the walker node
 * @version 1.0
 * @date 2024-11-24
 *
 * @copyright Copyright (c) 2024 Abhishek Avhad
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

/**
 * @brief Main function for the walker node
 *
 * Initializes ROS2 system, creates a WalkerNode instance, and runs the node.
 * The node implements a simple walker algorithm that moves the robot forward
 * until it encounters obstacles, then rotates to find a clear path.
 *
 * @param argc Number of command-line arguments
 * @param argv Array of command-line arguments
 * @return int Return status (0 for normal exit)
 */
int main(int argc, char** argv) {
  // Initialize ROS2 system
  rclcpp::init(argc, argv);

  // Create and initialize walker node
  auto node = std::make_shared<WalkerNode>();

  // Spin node to process callbacks
  rclcpp::spin(node);

  // Clean shutdown of ROS2 system
  rclcpp::shutdown();

  return 0;
}
