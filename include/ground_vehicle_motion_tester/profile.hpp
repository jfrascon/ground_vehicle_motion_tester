#pragma once

#include <vector>

namespace ground_vehicle_motion_tester
{
  /**
   * @brief Absolute bounds for the linear and angular components of one axis.
   *
   * Linear and angular motion each have their own acceleration and velocity bounds.
   * The bounds are inclusive and describe each component independently.
   */
  struct AxisLimits
  {
      /// @brief Inclusive absolute linear acceleration bound, in m/s².
      double a_max_abs{0.0};
      /// @brief Inclusive absolute linear velocity bound, in m/s.
      double v_max_abs{0.0};
      /// @brief Inclusive absolute angular acceleration bound, in rad/s².
      double alpha_max_abs{0.0};
      /// @brief Inclusive absolute angular velocity bound, in rad/s.
      double w_max_abs{0.0};
  };

  /** @brief Bounds shared by every profile in an ordered sequence. */
  struct Limits
  {
      /// @brief Linear and angular bounds of the X axis.
      AxisLimits x{};
      /// @brief Linear and angular bounds of the Y axis.
      AxisLimits y{};
      /// @brief Linear and angular bounds of the Z axis.
      AxisLimits z{};
  };

  /**
   * @brief Linear and angular velocity transitions requested on one axis.
   *
   * Linear motion uses the signed acceleration a and velocities v_init/v_end.
   * Angular motion uses the signed acceleration alpha and velocities w_init/w_end.
   * Each transition starts at the beginning of the profile. After reaching its final velocity,
   * that velocity is held until the profile ends.
   */
  struct AxisProfile
  {
      /// @brief Signed linear acceleration during the transition, in m/s².
      double a{0.0};
      /// @brief Linear velocity at the start of the profile, in m/s.
      double v_init{0.0};
      /// @brief Target linear velocity, held after the transition, in m/s.
      double v_end{0.0};
      /// @brief Signed angular acceleration during the transition, in rad/s².
      double alpha{0.0};
      /// @brief Angular velocity at the start of the profile, in rad/s.
      double w_init{0.0};
      /// @brief Target angular velocity, held after the transition, in rad/s.
      double w_end{0.0};
  };

  /**
   * @brief Simultaneous X, Y, and Z commands over one common duration.
   *
   * Each component can finish its velocity transition at a different time. The common duration
   * keeps those transitions in one profile and determines when the next profile begins.
   */
  struct Profile
  {
      /// @brief Common duration of all axis components, expressed in seconds.
      double duration{0.0};
      /// @brief Linear and angular transitions of the X axis over duration.
      AxisProfile x{};
      /// @brief Linear and angular transitions of the Y axis over duration.
      AxisProfile y{};
      /// @brief Linear and angular transitions of the Z axis over duration.
      AxisProfile z{};
  };

  /**
   * @brief Global bounds and profiles in their execution order.
   *
   * Consumers validate these supplied data against their motion model before using them.
   */
  struct ProfileSequence
  {
      /// @brief Component bounds applied to the entire sequence.
      Limits limits{};
      /// @brief List of profiles in their execution order.
      std::vector<Profile> profiles{};
  };
}  // namespace ground_vehicle_motion_tester
