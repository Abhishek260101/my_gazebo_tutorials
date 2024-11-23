#include "walker/walker_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WalkerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}