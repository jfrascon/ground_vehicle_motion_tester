#pragma once

#include <string>

#include "ground_vehicle_motion_tester/profile_validator.hpp"

namespace ground_vehicle_motion_tester
{
  /**
   * @brief Load and validate a sequence from a standalone YAML document.
   *
   * The root contains limits and profiles, without a ROS parameter wrapper. YAML field T
   * supplies Profile::duration. The loader
   * checks the schema first and then calls validate_profile_sequence() for the motion rules.
   * Executors and preview applications therefore receive the same interpretation of a file.
   *
   * @param yaml Exactly one YAML document containing the complete configuration.
   * @return A sequence that has passed every schema and mathematical check.
   * @throws std::invalid_argument Invalid YAML, schema, numeric values, or motion rules.
   */
  ProfileSequence load_profile_yaml(const std::string& yaml);

  /**
   * @brief Read a profile file and return its completely validated sequence.
   *
   * A consumer can call this function while initializing its configuration member. A failure
   * then stops construction before the consumer can execute or display invalid profiles.
   *
   * @param path Profile YAML path, used as supplied relative to the working directory.
   * @return The validated sequence produced by load_profile_yaml().
   * @throws std::invalid_argument File or validation failure, including the file path.
   */
  ProfileSequence load_profile_file(const std::string& path);
}  // namespace ground_vehicle_motion_tester
