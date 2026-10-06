#include "profile_executor_ros.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/create_timer.hpp>

#include "ground_vehicle_motion_tester/profile_yaml.hpp"

namespace ground_vehicle_motion_tester
{
  namespace
  {
    /**
     * @brief Describe a startup-only execution setting so it cannot change during a trial.
     * @param description Meaning of the parameter presented by ROS introspection.
     * @return A read-only parameter descriptor.
     */
    rcl_interfaces::msg::ParameterDescriptor startup_parameter(const std::string& description)
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor{};
      descriptor.description = description;
      descriptor.read_only = true;
      return descriptor;
    }

    /**
     * @brief Convert a finite positive publication rate to a representable timer period.
     * @param frequency Requested samples and publications per second.
     * @return Rounded nanosecond period for the ROS timer.
     * @throws std::invalid_argument Invalid rate or unrepresentable period.
     */
    std::chrono::nanoseconds publication_period(double frequency)
    {
      if(!std::isfinite(frequency) || frequency <= 0.0)
      {
        throw std::invalid_argument{"frequency must be finite and positive"};
      }
      const auto period{std::round(1e9L / static_cast<long double>(frequency))};
      if(!std::isfinite(period) || period < 1.0L || period > std::numeric_limits<std::int64_t>::max())
      {
        throw std::invalid_argument{"frequency produces an unrepresentable timer period"};
      }
      return std::chrono::nanoseconds{static_cast<std::int64_t>(period)};
    }
  }  // namespace

  ProfileExecutorROS::ProfileExecutorROS(const rclcpp::NodeOptions& options):
    rclcpp::Node{"ground_vehicle_motion_executor", options}
  {
    const auto file{
      declare_parameter<std::string>("profile_file", "", startup_parameter("Path to the standalone profile YAML."))};
    const auto frequency{
      declare_parameter<double>("frequency", 50.0, startup_parameter("Sampling and publication frequency in Hz."))};

    // Complete file validation and ramp preparation precede all command-related ROS entities.
    if(file.empty())
    {
      throw std::invalid_argument{"profile_file must name a profile YAML file"};
    }
    execution_.emplace(load_profile_file(file));
    const auto period{publication_period(frequency)};
    // Keep the relative topic name stable; ROS remappings select the actual destination.
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS{1});
    timer_ = rclcpp::create_timer(get_node_base_interface(),
                                  get_node_timers_interface(),
                                  get_clock(),
                                  rclcpp::Duration{period},
                                  [this]() {
                                    publish_sample();
                                  });
    RCLCPP_INFO(get_logger(),
                "Prepared profile '%s'; publishing on '%s' at %.3f Hz",
                file.c_str(),
                publisher_->get_topic_name(),
                frequency);
  }

  void ProfileExecutorROS::publish_sample()
  {
    const auto result{execution_->sample(get_clock()->now().nanoseconds())};
    if(result.status == ExecutionStatus::SkippedBackwardsTime)
    {
      if(!backwards_warning_issued_)
      {
        RCLCPP_WARN(get_logger(), "Clock moved backwards; discarding samples until it recovers");
        backwards_warning_issued_ = true;
      }
      return;
    }
    if(result.status == ExecutionStatus::Finished)
    {
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "Profile sequence finished; command publication has stopped");
      return;
    }
    backwards_warning_issued_ = false;
    const auto& state{*result.command};
    geometry_msgs::msg::Twist command{};
    command.linear.x = state.vx;
    command.linear.y = state.vy;
    command.angular.z = state.wz;
    publisher_->publish(command);
  }
}  // namespace ground_vehicle_motion_tester
