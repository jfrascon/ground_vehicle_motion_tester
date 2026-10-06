#include <exception>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "profile_executor_ros.hpp"

/**
 * @brief Start the ROS profile executor and retain the node after publication finishes.
 * @param argc ROS command-line argument count.
 * @param argv ROS arguments, including the required profile_file parameter.
 * @return Zero after normal shutdown, or one when initialization or execution fails.
 */
int main(int argc, char** argv)
{
  try
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ground_vehicle_motion_tester::ProfileExecutorROS>());
    rclcpp::shutdown();
    return 0;
  }
  catch(const std::exception& error)
  {
    std::cerr << "ground_vehicle_motion_executor: " << error.what() << '\n';
    if(rclcpp::ok())
    {
      rclcpp::shutdown();
    }
    return 1;
  }
}
