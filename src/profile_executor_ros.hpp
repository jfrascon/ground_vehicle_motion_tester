#pragma once

#include <optional>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ground_vehicle_motion_tester/profile_execution.hpp"

namespace ground_vehicle_motion_tester
{
  /**
   * @brief ROS adapter that publishes the shared evaluator's commands at a fixed target rate.
   *
   * The node's ROS clock drives both the timer and the sampled timestamp. ROS 2 owns clock
   * selection through use_sim_time. The default mutually exclusive callback group serializes
   * accesses to the execution cursor.
   */
  class ProfileExecutorROS: public rclcpp::Node
  {
    public:
      /**
       * @brief Load and prepare the YAML before creating the command publisher and timer.
       * @param options ROS node options, including initial parameters and remappings.
       * @throws std::invalid_argument Invalid profile file or execution parameters.
       */
      explicit ProfileExecutorROS(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

    private:
      /**
       * @brief Read one ROS timestamp, sample the cursor, and publish only an accepted command.
       * @note A backwards timestamp is discarded; completion cancels the timer.
       */
      void publish_sample();

      std::optional<ProfileExecution> execution_{};
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{};
      rclcpp::TimerBase::SharedPtr timer_{};
      bool backwards_warning_issued_{false};
  };
}  // namespace ground_vehicle_motion_tester
