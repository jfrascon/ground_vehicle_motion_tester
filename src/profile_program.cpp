#include "ground_vehicle_motion_tester/profile_program.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "ground_vehicle_motion_tester/profile_validator.hpp"

namespace ground_vehicle_motion_tester
{
  namespace
  {
    constexpr auto kNanosecondsPerSecond{1e9L};

    /**
     * @brief Prepare a validated component's coefficients and ramp deadline once.
     * @param profile Profile timing shared by the three components.
     * @param initial Initial velocity, the local straight-line intercept.
     * @param final Target velocity held after the ramp.
     * @param acceleration Signed ramp slope supplied by the YAML.
     * @return Owned coefficients and completion time; constant motion has no ramp.
     * @throws std::invalid_argument A changing velocity cannot fit the clock resolution.
     */
    PreparedTransition prepare_transition(const PreparedProfile& profile,
                                          double initial,
                                          double final,
                                          double acceleration)
    {
      PreparedTransition result{initial, final, acceleration, profile.start_time, profile.start_ns};
      if(initial == final)
      {
        return result;
      }
      const auto transition{(static_cast<long double>(final) - initial) / acceleration};
      // Preserve the validator's rounding allowance at T_acc = T without relaxing bounds.
      const auto transition_time{static_cast<double>(std::min(transition, static_cast<long double>(profile.duration)))};
      result.end_time = profile.start_time + transition_time;
      result.end_ns = profile_time_to_nanoseconds(result.end_time);
      if(result.end_ns <= profile.start_ns)
      {
        throw std::invalid_argument{"profile ramp is shorter than the nanosecond clock resolution"};
      }
      return result;
    }

    /**
     * @brief Apply stored local ramp coefficients or return the exact final-velocity hold.
     * @param transition Coefficients prepared before execution.
     * @param elapsed_ns Global relative timestamp selecting the ramp or hold.
     * @param local_time Seconds elapsed inside the selected profile.
     * @return Current velocity and acceleration.
     */
    std::pair<double, double> evaluate_transition(const PreparedTransition& transition,
                                                  std::int64_t elapsed_ns,
                                                  double local_time)
    {
      if(elapsed_ns >= transition.end_ns)
      {
        return {transition.final, 0.0};
      }
      const auto velocity{static_cast<long double>(transition.initial) +
                          (static_cast<long double>(transition.acceleration) * local_time)};
      // Clock rounding must not move the interpolated velocity beyond either endpoint.
      return {std::clamp(static_cast<double>(velocity),
                         std::min(transition.initial, transition.final),
                         std::max(transition.initial, transition.final)),
              transition.acceleration};
    }
  }  // namespace

  std::int64_t profile_time_to_nanoseconds(double seconds)
  {
    const auto rounded{std::round(static_cast<long double>(seconds) * kNanosecondsPerSecond)};
    if(!std::isfinite(rounded) || seconds < 0.0 || rounded > std::numeric_limits<std::int64_t>::max())
    {
      throw std::invalid_argument{"profile time cannot be represented by the nanosecond clock"};
    }
    return static_cast<std::int64_t>(rounded);
  }

  ProfileProgram::ProfileProgram(const ProfileSequence& sequence)
  {
    validate_profile_sequence(sequence);
    profiles_.reserve(sequence.profiles.size());
    auto start{0.0};
    for(const auto& profile: sequence.profiles)
    {
      const auto end{start + profile.duration};
      PreparedProfile prepared{start,
                               end,
                               profile.duration,
                               profile_time_to_nanoseconds(start),
                               profile_time_to_nanoseconds(end)};
      if(prepared.end_ns <= prepared.start_ns)
      {
        throw std::invalid_argument{"profile duration is shorter than the nanosecond clock resolution"};
      }
      prepared.x = prepare_transition(prepared, profile.x.v_init, profile.x.v_end, profile.x.a);
      prepared.y = prepare_transition(prepared, profile.y.v_init, profile.y.v_end, profile.y.a);
      prepared.z = prepare_transition(prepared, profile.z.w_init, profile.z.w_end, profile.z.alpha);
      profiles_.push_back(prepared);
      start = end;
    }
  }

  const std::vector<PreparedProfile>& ProfileProgram::profiles() const noexcept
  {
    return profiles_;
  }

  std::int64_t ProfileProgram::duration_ns() const noexcept
  {
    return profiles_.back().end_ns;
  }

  MotionState ProfileProgram::evaluate(std::size_t index, std::int64_t elapsed_ns) const
  {
    const auto& profile{profiles_.at(index)};
    if(elapsed_ns < profile.start_ns || elapsed_ns > profile.end_ns)
    {
      throw std::invalid_argument{"evaluation time is outside the selected profile"};
    }
    // Subtract integers first so short ramps retain precision after a long earlier profile.
    const auto local{
      std::min(static_cast<double>(elapsed_ns - profile.start_ns) * kSecondsPerNanosecond, profile.duration)};
    const auto x{evaluate_transition(profile.x, elapsed_ns, local)};
    const auto y{evaluate_transition(profile.y, elapsed_ns, local)};
    const auto z{evaluate_transition(profile.z, elapsed_ns, local)};
    return {x.first, y.first, z.first, x.second, y.second, z.second};
  }
}  // namespace ground_vehicle_motion_tester
