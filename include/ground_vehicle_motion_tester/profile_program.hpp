#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "ground_vehicle_motion_tester/profile.hpp"

namespace ground_vehicle_motion_tester
{
  /// @brief Convert elapsed clock nanoseconds to seconds without using an absolute epoch.
  inline constexpr auto kSecondsPerNanosecond{1e-9};

  /** @brief Precomputed ramp coefficients and completion time for one velocity component. */
  struct PreparedTransition
  {
      double initial{0.0};
      double final{0.0};
      // In local profile time, acceleration is the slope and initial is the intercept.
      double acceleration{0.0};
      double end_time{0.0};
      std::int64_t end_ns{0};
  };

  /** @brief Accumulated profile interval and its three permitted ground vehicle transitions. */
  struct PreparedProfile
  {
      double start_time{0.0};
      double end_time{0.0};
      double duration{0.0};
      std::int64_t start_ns{0};
      std::int64_t end_ns{0};
      PreparedTransition x{};
      PreparedTransition y{};
      PreparedTransition z{};
  };

  /** @brief Instantaneous commanded body velocities and their profile accelerations. */
  struct MotionState
  {
      double vx{0.0};
      double vy{0.0};
      double wz{0.0};
      double ax{0.0};
      double ay{0.0};
      double awz{0.0};
  };

  /**
   * @brief Round a nonnegative profile time to the shared nanosecond clock.
   * @param seconds Relative sequence time or positive duration, in seconds.
   * @return Representable signed nanosecond value.
   * @throws std::invalid_argument Nonfinite, negative, or out-of-range time.
   */
  std::int64_t profile_time_to_nanoseconds(double seconds);

  /**
   * @brief Validate and prepare the immutable mathematical program shared by both consumers.
   *
   * Every ramp completion is computed once. Evaluation then needs only the active profile,
   * local time, and comparisons with prepared ramp deadlines; it performs no YAML parsing.
   */
  class ProfileProgram
  {
    public:
      /**
       * @brief Prepare owned profile data without retaining references to the caller's vector.
       * @param sequence Complete profile sequence to validate.
       * @throws std::invalid_argument Invalid data or unrepresentable timing.
       */
      explicit ProfileProgram(const ProfileSequence& sequence);

      /** @brief Return immutable intervals and transitions in execution order. */
      const std::vector<PreparedProfile>& profiles() const noexcept;

      /** @brief Return the accumulated duration in the shared nanosecond clock. */
      std::int64_t duration_ns() const noexcept;

      /**
       * @brief Evaluate a prepared profile's ramps or final-velocity holds.
       *
       * Subtract the integer profile start before converting local time to seconds. Both
       * execution and preview therefore use one clock for phase selection and interpolation.
       *
       * @param index Active profile index selected by the consumer.
       * @param elapsed_ns Time from the sequence origin, used for both phases and velocities.
       * @return Commanded velocities and accelerations, including the final endpoint.
       * @throws std::out_of_range Index is outside the prepared vector.
       * @throws std::invalid_argument Time is invalid or outside the selected interval.
       */
      MotionState evaluate(std::size_t index, std::int64_t elapsed_ns) const;

    private:
      std::vector<PreparedProfile> profiles_{};
  };
}  // namespace ground_vehicle_motion_tester
