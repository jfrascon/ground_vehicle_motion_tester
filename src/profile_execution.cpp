#include "ground_vehicle_motion_tester/profile_execution.hpp"

namespace ground_vehicle_motion_tester
{
  ProfileExecution::ProfileExecution(const ProfileSequence& sequence): program_{sequence}
  {}

  ExecutionResult ProfileExecution::sample(std::int64_t stamp_ns)
  {
    const auto& profiles{program_.profiles()};
    if(index_ == profiles.size())
    {
      return {ExecutionStatus::Finished, std::nullopt, index_};
    }
    if(last_accepted_ns_ && stamp_ns < *last_accepted_ns_)
    {
      // Retain the last valid state until this same clock reaches that time again.
      return {ExecutionStatus::SkippedBackwardsTime, std::nullopt, index_};
    }
    if(!origin_ns_)
    {
      origin_ns_ = stamp_ns;
    }

    // Subtract integer timestamps before converting to seconds. Unsigned subtraction also
    // handles a very large forward jump without overflowing a signed elapsed-time value.
    const auto elapsed{static_cast<std::uint64_t>(stamp_ns) - static_cast<std::uint64_t>(*origin_ns_)};
    last_accepted_ns_ = stamp_ns;
    while(index_ < profiles.size() && elapsed >= static_cast<std::uint64_t>(profiles[index_].end_ns))
    {
      ++index_;
    }
    if(index_ == profiles.size())
    {
      // Finishing does not invent an extra zero or final-velocity publication.
      return {ExecutionStatus::Finished, std::nullopt, index_};
    }

    return {ExecutionStatus::SampleReady, program_.evaluate(index_, static_cast<std::int64_t>(elapsed)), index_};
  }
}  // namespace ground_vehicle_motion_tester
