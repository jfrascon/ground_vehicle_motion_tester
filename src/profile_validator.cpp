#include "ground_vehicle_motion_tester/profile_validator.hpp"

#include <cmath>
#include <string>

namespace ground_vehicle_motion_tester
{
  namespace
  {
    /**
     * @brief Stop validation at the first failed condition with a field-specific message.
     * @param condition Whether the rule is satisfied.
     * @param path YAML path identifying the invalid field or collection.
     * @param reason Description of the rule needed to fix the configuration.
     * @throws std::invalid_argument If condition is false.
     */
    void require(bool condition, const std::string& path, const std::string& reason)
    {
      if(!condition)
      {
        throw std::invalid_argument{path + ": " + reason};
      }
    }

    /**
     * @brief Reject NaN and infinities before they enter comparisons or arithmetic.
     * @param value Number to check.
     * @param path Field path used in the error.
     * @throws std::invalid_argument If value is not finite.
     */
    void check_finite(double value, const std::string& path)
    {
      require(std::isfinite(value), path, "must be finite");
    }

    /**
     * @brief Check an absolute bound; zero is valid for a restricted component.
     * @param value Bound that must be finite and nonnegative.
     * @param path Field path used in the error.
     * @throws std::invalid_argument If the bound is invalid.
     */
    void check_limit(double value, const std::string& path)
    {
      check_finite(value, path);
      require(value >= 0.0, path, "must be nonnegative");
    }

    /**
     * @brief Reject any enabled component outside the planar X/Y translation and Z rotation.
     * @param value Component or bound that must be exactly zero.
     * @param path Field path used in the error.
     * @throws std::invalid_argument If value is nonfinite or nonzero.
     */
    void check_zero(double value, const std::string& path)
    {
      check_finite(value, path);
      require(value == 0.0, path, "must be zero for planar motion");
    }

    /**
     * @brief Validate active bounds and require inactive bounds to stay zero on one axis.
     * @param limits All four bounds of the axis.
     * @param path Axis path under limits.
     * @param angular True for Z rotation; false for X or Y translation.
     * @throws std::invalid_argument If any bound fails its applicable rule.
     */
    void check_limits(const AxisLimits& limits, const std::string& path, bool angular)
    {
      if(angular)
      {
        check_zero(limits.a_max_abs, path + ".a_max_abs");
        check_zero(limits.v_max_abs, path + ".v_max_abs");
        check_limit(limits.alpha_max_abs, path + ".alpha_max_abs");
        check_limit(limits.w_max_abs, path + ".w_max_abs");
      }
      else
      {
        check_limit(limits.a_max_abs, path + ".a_max_abs");
        check_limit(limits.v_max_abs, path + ".v_max_abs");
        check_zero(limits.alpha_max_abs, path + ".alpha_max_abs");
        check_zero(limits.w_max_abs, path + ".w_max_abs");
      }
    }

    /**
     * @brief Check that a signed constant acceleration reaches the target within the profile.
     *
     * Equal endpoints describe constant motion and require zero acceleration. Otherwise,
     * zero acceleration or the wrong sign cannot reach the target. The transition may finish
     * before T because the final velocity is then held for the remaining profile duration.
     *
     * @param acceleration Signed linear or angular acceleration.
     * @param initial Initial velocity of the active component.
     * @param final Target velocity of the active component.
     * @param duration Positive duration already validated by the caller.
     * @param acceleration_limit Validated absolute acceleration bound.
     * @param velocity_limit Validated absolute velocity bound.
     * @param path Axis path under the current profile.
     * @param angular Select angular field names for diagnostics when true.
     * @throws std::invalid_argument Invalid numbers, exceeded bounds, or unreachable target.
     */
    void check_motion(double acceleration,
                      double initial,
                      double final,
                      double duration,
                      double acceleration_limit,
                      double velocity_limit,
                      const std::string& path,
                      bool angular)
    {
      const std::string acceleration_path{path + (angular ? ".alpha" : ".a")};
      const std::string initial_path{path + (angular ? ".w_init" : ".v_init")};
      const std::string final_path{path + (angular ? ".w_end" : ".v_end")};
      check_finite(acceleration, acceleration_path);
      check_finite(initial, initial_path);
      check_finite(final, final_path);
      require(std::abs(acceleration) <= acceleration_limit, acceleration_path, "exceeds its absolute limit");
      require(std::abs(initial) <= velocity_limit, initial_path, "exceeds its absolute limit");
      require(std::abs(final) <= velocity_limit, final_path, "exceeds its absolute limit");

      // Handle constant motion separately: its transition time would be the undefined 0/0.
      if(initial == final)
      {
        require(acceleration == 0.0, acceleration_path, "must be zero when initial and final velocities are equal");
        return;
      }

      require(acceleration != 0.0, acceleration_path, "must be nonzero when velocity changes");
      // Wider arithmetic avoids overflowing the difference between two finite double values.
      const auto delta{static_cast<long double>(final) - static_cast<long double>(initial)};
      require((delta > 0.0L) == (acceleration > 0.0),
              acceleration_path,
              "has the wrong sign to reach the final velocity");
      const auto transition_time{delta / static_cast<long double>(acceleration)};
      require(std::isfinite(transition_time) && transition_time > 0.0L,
              acceleration_path,
              "must produce a finite, positive transition time");
      // Compare times relatively so rounding at T_acc = T does not reject a valid ramp.
      // This allowance does not relax acceleration or velocity bounds.
      const auto relative_excess{(transition_time - duration) / duration};
      require(relative_excess <= kTransitionTimeRelativeTolerance,
              acceleration_path,
              "transition time (final - initial) / acceleration exceeds T");
    }

    /**
     * @brief Validate the permitted motion and the zero-only components of one profile axis.
     * @param axis Initial/final velocities and signed accelerations.
     * @param limits Bounds already validated for this axis.
     * @param duration Common positive profile duration.
     * @param path Current profile and axis path.
     * @param angular True for Z rotation; false for X or Y translation.
     * @throws std::invalid_argument A forbidden component or an invalid active transition.
     */
    void check_axis(const AxisProfile& axis,
                    const AxisLimits& limits,
                    double duration,
                    const std::string& path,
                    bool angular)
    {
      if(angular)
      {
        check_zero(axis.a, path + ".a");
        check_zero(axis.v_init, path + ".v_init");
        check_zero(axis.v_end, path + ".v_end");
        check_motion(axis.alpha, axis.w_init, axis.w_end, duration, limits.alpha_max_abs, limits.w_max_abs, path, true);
      }
      else
      {
        check_zero(axis.alpha, path + ".alpha");
        check_zero(axis.w_init, path + ".w_init");
        check_zero(axis.w_end, path + ".w_end");
        check_motion(axis.a, axis.v_init, axis.v_end, duration, limits.a_max_abs, limits.v_max_abs, path, false);
      }
    }

    /**
     * @brief Prevent an unconfigured velocity jump between two consecutive profiles.
     * @param previous Validated final velocity of the preceding profile.
     * @param next Validated initial velocity of the current profile.
     * @param path Current initial-velocity field used in the error.
     * @throws std::invalid_argument Difference exceeds the absolute continuity tolerance.
     */
    void check_continuity(double previous, double next, const std::string& path)
    {
      const auto delta{static_cast<long double>(next) - static_cast<long double>(previous)};
      require(std::abs(delta) <= kVelocityContinuityTolerance,
              path,
              "does not match the previous profile's final velocity");
    }
  }  // namespace

  void validate_profile_sequence(const ProfileSequence& sequence)
  {
    // Validate shared bounds before using them to check any individual profile.
    check_limits(sequence.limits.x, "limits.x", false);
    check_limits(sequence.limits.y, "limits.y", false);
    check_limits(sequence.limits.z, "limits.z", true);
    require(!sequence.profiles.empty(), "profiles", "must contain at least one profile");

    for(std::size_t index{0}; index < sequence.profiles.size(); ++index)
    {
      const auto& profile{sequence.profiles[index]};
      const std::string path{"profiles[" + std::to_string(index) + "]"};
      check_finite(profile.duration, path + ".T");
      require(profile.duration > 0.0, path + ".T", "must be positive");
      check_axis(profile.x, sequence.limits.x, profile.duration, path + ".x", false);
      check_axis(profile.y, sequence.limits.y, profile.duration, path + ".y", false);
      check_axis(profile.z, sequence.limits.z, profile.duration, path + ".z", true);

      // There is no previous profile for index zero. Its initial velocity remains unrestricted
      // within the configured bounds, just as the last profile may finish while moving.
      if(index > 0)
      {
        const auto& previous{sequence.profiles[index - 1]};
        check_continuity(previous.x.v_end, profile.x.v_init, path + ".x.v_init");
        check_continuity(previous.y.v_end, profile.y.v_init, path + ".y.v_init");
        check_continuity(previous.z.w_end, profile.z.w_init, path + ".z.w_init");
      }
    }
  }
}  // namespace ground_vehicle_motion_tester
