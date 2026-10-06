#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "ground_vehicle_motion_tester/profile_program.hpp"

namespace ground_vehicle_motion_tester
{
  /** @brief Outcome of one requested execution sample, independent of its ROS publication. */
  enum class ExecutionStatus
  {
    SampleReady,
    SkippedBackwardsTime,
    Finished,
  };

  /** @brief A command exists only for an accepted time inside the configured sequence. */
  struct ExecutionResult
  {
      ExecutionStatus status{ExecutionStatus::Finished};
      std::optional<MotionState> command{};
      std::size_t profile_index{0};
  };

  /**
   * @brief Advance through prepared profiles using timestamps from one caller-selected clock.
   *
   * The first accepted sample establishes t0. A backwards sample changes neither this origin
   * nor the index nor the last accepted timestamp. The caller publishes only SampleReady.
   * Call sample serially; the cursor deliberately has no internal thread synchronization.
   */
  class ProfileExecution
  {
    public:
      /**
       * @brief Validate and prepare all intervals and ramps before any sampling.
       * @param sequence Complete profile sequence owned independently after preparation.
       * @throws std::invalid_argument Invalid profiles or unrepresentable profile timing.
       */
      explicit ProfileExecution(const ProfileSequence& sequence);

      /**
       * @brief Select and evaluate the current profile using actual elapsed time.
       * @param stamp_ns Timestamp from the same clock used for all previous calls.
       * @return A ready command, a discarded backwards sample, or permanent completion.
       * @note A delayed call can skip multiple profiles. Endpoints belong to the next interval.
       * @note Completion returns no command, even when the last velocity is nonzero.
       */
      ExecutionResult sample(std::int64_t stamp_ns);

    private:
      ProfileProgram program_;
      std::optional<std::int64_t> origin_ns_{};
      std::optional<std::int64_t> last_accepted_ns_{};
      // Reaching the vector size is permanent completion, so no separate flag is needed.
      std::size_t index_{0};
  };
}  // namespace ground_vehicle_motion_tester
