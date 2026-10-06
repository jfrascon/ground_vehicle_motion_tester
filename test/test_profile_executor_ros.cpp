#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl/time.h>
#include <rclcpp/rclcpp.hpp>

#include "profile_executor_ros.hpp"

namespace gvmt = ground_vehicle_motion_tester;

namespace
{
  /** @brief Isolate ROS test commands from any robot command topics and DDS domains. */
  class ExecutorROS: public ::testing::Test
  {
    protected:
      /** @brief Create a private context, observer, and serial executor for one test. */
      void SetUp() override
      {
        rclcpp::InitOptions init_options{};
        init_options.set_domain_id(177);
        context_ = std::make_shared<rclcpp::Context>();
        context_->init(0, nullptr, init_options);
        rclcpp::NodeOptions options{};
        options.context(context_).enable_rosout(false);
        observer_ = std::make_shared<rclcpp::Node>("motion_executor_observer", options);
        subscription_ = observer_->create_subscription<
          geometry_msgs::msg::Twist>("/motion_executor_test/commands",
                                     rclcpp::QoS{100},
                                     [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
                                       commands_.push_back(*message);
                                     });
        rclcpp::ExecutorOptions executor_options{};
        executor_options.context = context_;
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
        executor_->add_node(observer_);
      }

      /** @brief Shut down the private ROS context after removing its execution references. */
      void TearDown() override
      {
        executor_.reset();
        node_.reset();
        subscription_.reset();
        observer_.reset();
        context_->shutdown("executor test finished");
        context_.reset();
      }

      /**
       * @brief Build one node with explicit file, clock, frequency, and isolated command topic.
       * @param simulated Whether ROS should activate its simulation clock.
       * @param frequency Publication rate under test.
       * @param file Profile file under test.
       * @return A configured node whose timer has not yet been serviced by the executor.
       */
      std::shared_ptr<gvmt::ProfileExecutorROS> make_node(bool simulated,
                                                          double frequency = 100.0,
                                                          const std::string& file = EXECUTOR_TEST_PROFILE)
      {
        rclcpp::NodeOptions options{};
        options.context(context_).enable_rosout(false).use_clock_thread(false);
        options.arguments({"--ros-args", "-r", "cmd_vel:=/motion_executor_test/commands"});
        options.parameter_overrides({rclcpp::Parameter{"profile_file", file},
                                     rclcpp::Parameter{"frequency", frequency},
                                     rclcpp::Parameter{"use_sim_time", simulated}});
        return std::make_shared<gvmt::ProfileExecutorROS>(options);
      }

      /**
       * @brief Service ROS work while wall time passes, without advancing simulated time.
       * @param duration Wall-time interval used only to wait for middleware and callbacks.
       */
      void spin_for(std::chrono::milliseconds duration)
      {
        const auto deadline{std::chrono::steady_clock::now() + duration};
        do
        {
          executor_->spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds{1});
        } while(std::chrono::steady_clock::now() < deadline);
      }

      /**
       * @brief Set the ROS clock through RCL's override API, without a test wall timer.
       * @param nanoseconds The new simulation timestamp.
       */
      void set_time(std::int64_t nanoseconds)
      {
        ASSERT_TRUE(node_->get_clock()->ros_time_is_active());
        ASSERT_EQ(rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), nanoseconds), RCL_RET_OK);
        spin_for(std::chrono::milliseconds{35});
      }

      std::shared_ptr<rclcpp::Context> context_{};
      std::shared_ptr<rclcpp::Node> observer_{};
      std::shared_ptr<gvmt::ProfileExecutorROS> node_{};
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_{};
      std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_{};
      std::vector<geometry_msgs::msg::Twist> commands_{};
  };
}  // namespace

/** @test Node initialization rejects malformed execution settings before creating commands. */
TEST_F(ExecutorROS, RejectsBadStartupSettings)
{
  EXPECT_THROW(make_node(true, 0.0), std::invalid_argument);
  EXPECT_THROW(make_node(true, std::numeric_limits<double>::infinity()), std::invalid_argument);
  EXPECT_THROW(make_node(true, 100.0, ""), std::invalid_argument);
  EXPECT_THROW(make_node(true, 100.0, "/tmp/no_executor_profile.yaml"), std::invalid_argument);
  EXPECT_TRUE(commands_.empty());
}

/** @test Real-time mode still uses the node's ROS clock, with ROS managing its source. */
TEST_F(ExecutorROS, PublishesWithTheNormalROSClockAndStops)
{
  node_ = make_node(false);
  EXPECT_EQ(node_->get_clock()->get_clock_type(), RCL_ROS_TIME);
  EXPECT_FALSE(node_->get_clock()->ros_time_is_active());
  executor_->add_node(node_);
  spin_for(std::chrono::milliseconds{400});
  ASSERT_FALSE(commands_.empty());
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  const auto stopped_count{commands_.size()};
  spin_for(std::chrono::milliseconds{80});
  EXPECT_EQ(commands_.size(), stopped_count);
}

/** @test ROS-time pause, ramp evaluation, backwards discard, recovery, and completion agree. */
TEST_F(ExecutorROS, SimulationClockControlsTheTimerAndTheSampledTime)
{
  node_ = make_node(true);
  executor_->add_node(node_);
  spin_for(std::chrono::milliseconds{200});
  ASSERT_EQ(observer_->count_publishers("/motion_executor_test/commands"), 1U);
  ASSERT_TRUE(commands_.empty());

  set_time(10'000'000);
  ASSERT_EQ(commands_.size(), 1U);
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  spin_for(std::chrono::milliseconds{60});
  EXPECT_EQ(commands_.size(), 1U);

  set_time(20'000'000);
  ASSERT_EQ(commands_.size(), 2U);
  EXPECT_NEAR(commands_.back().linear.x, 0.01, 1e-12);
  set_time(30'000'000);
  ASSERT_EQ(commands_.size(), 3U);
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.02);
  set_time(50'000'000);
  ASSERT_EQ(commands_.size(), 4U);
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.02);
  set_time(100'000'000);
  ASSERT_EQ(commands_.size(), 5U);
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);

  set_time(20'000'000);
  EXPECT_EQ(commands_.size(), 5U);
  set_time(50'000'000);
  EXPECT_EQ(commands_.size(), 5U);
  set_time(120'000'000);
  ASSERT_EQ(commands_.size(), 6U);
  EXPECT_DOUBLE_EQ(commands_.back().linear.x, 0.0);
  set_time(160'000'000);
  EXPECT_EQ(commands_.size(), 6U);
  set_time(250'000'000);
  EXPECT_EQ(commands_.size(), 6U);
  for(const auto& command: commands_)
  {
    EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(command.linear.z, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.x, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.y, 0.0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
  }
}
