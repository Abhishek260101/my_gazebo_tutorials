#ifndef WALKER_STATE_HPP_
#define WALKER_STATE_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include <vector>

// Forward declaration
class WalkerNode;

/**
 * @class WalkerState
 * @brief Abstract base class for walker states
 *
 * This class defines the interface for different states in the walker's
 * state machine. Each state handles laser scan data differently and
 * produces appropriate velocity commands.
 */
class WalkerState {
 public:
    /**
     * @brief Virtual destructor
     */
    virtual ~WalkerState() = default;
    
    /**
     * @brief Processes laser scan data to produce velocity commands
     * @param scan Shared pointer to laser scan message
     * @return Velocity command for the robot
     */
    virtual geometry_msgs::msg::Twist process_scan(
        const sensor_msgs::msg::LaserScan::SharedPtr scan) = 0;
    
    /**
     * @brief Gets the name of the current state
     * @return String representing the state name
     */
    virtual std::string get_state_name() const = 0;

    /**
     * @brief Utility function to check if path is blocked
     * @param ranges Vector of scan ranges
     * @param min_distance Minimum safe distance
     * @return true if path is blocked, false otherwise
     */
    static bool is_path_blocked(const std::vector<float>& ranges, 
                              double min_distance);

 protected:
    WalkerNode* context_;  ///< Pointer to the WalkerNode context
    
    /**
     * @brief Protected constructor for state initialization
     * @param context Pointer to WalkerNode context
     */
    explicit WalkerState(WalkerNode* context) : context_(context) {}
};

/**
 * @class ForwardState
 * @brief State for moving forward
 *
 * This state moves the robot forward until an obstacle is detected,
 * then transitions to the rotating state.
 */
class ForwardState : public WalkerState {
 public:
    /**
     * @brief Constructor for forward state
     * @param context Pointer to WalkerNode context
     */
    explicit ForwardState(WalkerNode* context) : WalkerState(context) {}
    
    geometry_msgs::msg::Twist process_scan(
        const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
    
    std::string get_state_name() const override {
        return "Forward";
    }

    /**
     * @brief Creates a forward movement command
     * @param linear_vel Desired forward velocity
     * @return Twist message for forward movement
     */
    static geometry_msgs::msg::Twist create_forward_command(double linear_vel);
};

/**
 * @class RotatingState
 * @brief State for rotating to avoid obstacles
 *
 * This state rotates the robot until the path is clear,
 * then transitions back to the forward state.
 */
class RotatingState : public WalkerState {
 public:
    /**
     * @brief Constructor for rotating state
     * @param context Pointer to WalkerNode context
     * @param rotate_clockwise Direction of rotation
     */
    explicit RotatingState(WalkerNode* context, bool rotate_clockwise) 
        : WalkerState(context), rotate_clockwise_(rotate_clockwise) {}
    
    geometry_msgs::msg::Twist process_scan(
        const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
    
    std::string get_state_name() const override {
        return "Rotating";
    }

    /**
     * @brief Creates a rotation command
     * @param angular_vel Angular velocity
     * @param clockwise Direction of rotation
     * @return Twist message for rotation
     */
    static geometry_msgs::msg::Twist create_rotation_command(
        double angular_vel, bool clockwise);

    /**
     * @brief Gets the current rotation direction
     * @return true if rotating clockwise
     */
    bool is_rotating_clockwise() const { return rotate_clockwise_; }

 private:
    bool rotate_clockwise_;  ///< Flag for rotation direction
};

#endif  // WALKER_STATE_HPP_