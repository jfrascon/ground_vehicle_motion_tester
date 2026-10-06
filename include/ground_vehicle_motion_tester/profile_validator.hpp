#pragma once

#include <limits>
#include <stdexcept>

#include "ground_vehicle_motion_tester/profile.hpp"

namespace ground_vehicle_motion_tester
{
  /// @brief Absolute rounding allowance for consecutive velocities, in m/s or rad/s.
  inline constexpr auto kVelocityContinuityTolerance{1e-9};
  /// @brief Relative rounding allowance when the calculated transition time equals T.
  inline constexpr auto kTransitionTimeRelativeTolerance{32.0 * std::numeric_limits<double>::epsilon()};

  /**
   * @brief Validate the full sequence before a consumer uses any of its profiles.
   *
   * This validator applies the ground vehicle model to the general axis profiles.
   * X and Y use the signed acceleration a and velocities v_init/v_end.
   * Z uses the signed angular acceleration alpha and velocities w_init/w_end.
   * The other components and their bounds must be zero for planar motion.
   *
   * Check numeric values, planar components, bounds, positive durations, reachable final
   * velocities, and continuity between consecutive profiles. For a velocity change, the
   * signed acceleration must reach the target within T; equal velocities require zero
   * acceleration. Initial and final sequence velocities may be nonzero.
   *
   * Continuity allows kVelocityContinuityTolerance. The transition-time boundary allows
   * kTransitionTimeRelativeTolerance. Bounds and required zeros are checked directly.
   * This function does not modify the sequence and does not require ROS or YAML.
   *
   * @param sequence Global limits and an ordered, nonempty list of profiles.
   * @throws std::invalid_argument The first failed rule, with its field path.
   */
  void validate_profile_sequence(const ProfileSequence& sequence);
}  // namespace ground_vehicle_motion_tester
