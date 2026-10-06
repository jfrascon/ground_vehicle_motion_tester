#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>

#include "ground_vehicle_motion_tester/profile_execution.hpp"
#include "ground_vehicle_motion_tester/profile_yaml.hpp"

namespace gvmt = ground_vehicle_motion_tester;

namespace
{
  /** @brief Build a ramp, deceleration, and final hold with distinct execution intervals. */
  gvmt::ProfileSequence sequence()
  {
    gvmt::ProfileSequence result{};
    result.limits.x = {0.25, 0.5};
    gvmt::Profile forward{};
    forward.duration = 4.0;
    forward.x = {0.25, 0.0, 0.5};
    gvmt::Profile deceleration{};
    deceleration.duration = 2.0;
    deceleration.x = {-0.25, 0.5, 0.0};
    gvmt::Profile hold{};
    hold.duration = 1.0;
    result.profiles = {forward, deceleration, hold};
    return result;
  }
}  // namespace

/** @test The timestamp selecting a ramp must also determine its velocity. */
TEST(ProfileProgram, UsesOneTimeForRampSelectionAndVelocity)
{
  const gvmt::ProfileProgram program{sequence()};
  const auto state{program.evaluate(0, 1'000'000'000)};
  EXPECT_DOUBLE_EQ(state.vx, 0.25);
}

/** @test Local ramp evaluation retains nanosecond precision after a long constant profile. */
TEST(ProfileProgram, SubtractsProfileStartBeforeConvertingToSeconds)
{
  auto input{sequence()};
  gvmt::Profile initial{};
  initial.duration = 9'000'000.0;
  input.profiles.insert(input.profiles.begin(), initial);
  const gvmt::ProfileProgram program{input};
  const auto stamp{program.profiles()[1].start_ns};
  EXPECT_DOUBLE_EQ(program.evaluate(1, stamp).vx, 0.0);
  EXPECT_DOUBLE_EQ(program.evaluate(1, stamp + 1).vx, 0.25 * gvmt::kSecondsPerNanosecond);
  EXPECT_DOUBLE_EQ(program.evaluate(1, stamp + 1'000'000'000).vx, 0.25);
}

/** @test Fractional start times use the same rounded clock for ramp phases and local time. */
TEST(ProfileProgram, UsesRoundedProfileStartForLocalRampTime)
{
  auto input{sequence()};
  gvmt::Profile initial{};
  initial.duration = 0.5000000004;
  input.profiles.insert(input.profiles.begin(), initial);
  const gvmt::ProfileProgram program{input};
  const auto stamp{program.profiles()[1].start_ns};
  EXPECT_EQ(stamp, 500'000'000);
  EXPECT_DOUBLE_EQ(program.evaluate(1, stamp).vx, 0.0);
  EXPECT_DOUBLE_EQ(program.evaluate(1, stamp + 1).vx, 0.25 * gvmt::kSecondsPerNanosecond);
}

/** @test A prepared profile accepts its endpoints and rejects unrelated intervals or indices. */
TEST(ProfileProgram, ChecksSelectedIntervalAndIndex)
{
  const gvmt::ProfileProgram program{sequence()};
  EXPECT_THROW(program.evaluate(0, -1), std::invalid_argument);
  EXPECT_THROW(program.evaluate(0, 4'000'000'001), std::invalid_argument);
  EXPECT_THROW(program.evaluate(3, 0), std::out_of_range);
  EXPECT_DOUBLE_EQ(program.evaluate(0, 4'000'000'000).vx, 0.5);
  EXPECT_DOUBLE_EQ(program.evaluate(1, 4'000'000'000).vx, 0.5);
}

/** @test The first callback establishes the origin, independent of construction or epoch. */
TEST(ProfileExecution, StartsAtTheFirstSampleAndSubtractsIntegerTimestamps)
{
  gvmt::ProfileExecution execution{sequence()};
  constexpr std::int64_t epoch{1'700'000'000'000'000'000};
  const auto first{execution.sample(epoch)};
  ASSERT_EQ(first.status, gvmt::ExecutionStatus::SampleReady);
  ASSERT_TRUE(first.command);
  EXPECT_DOUBLE_EQ(first.command->vx, 0.0);
  const auto next{execution.sample(epoch + 500'000'000)};
  ASSERT_TRUE(next.command);
  EXPECT_DOUBLE_EQ(next.command->vx, 0.125);
}

/** @test Ramp completion selects the exact final velocity and zero acceleration. */
TEST(ProfileExecution, DistinguishesTheRampAndTheHold)
{
  gvmt::ProfileExecution execution{sequence()};
  execution.sample(0);
  const auto ramp{execution.sample(1'000'000'000)};
  const auto hold{execution.sample(2'000'000'000)};
  ASSERT_TRUE(ramp.command);
  ASSERT_TRUE(hold.command);
  EXPECT_DOUBLE_EQ(ramp.command->vx, 0.25);
  EXPECT_DOUBLE_EQ(ramp.command->ax, 0.25);
  EXPECT_DOUBLE_EQ(hold.command->vx, 0.5);
  EXPECT_DOUBLE_EQ(hold.command->ax, 0.0);
}

/** @test Each component independently finishes its ramp within the common duration. */
TEST(ProfileExecution, EvaluatesIndependentAxisRampTimes)
{
  auto input{sequence()};
  input.profiles.resize(1);
  input.limits.y = {0.5, 0.5};
  input.limits.z = {0.0, 0.0, 1.0, 0.5};
  input.profiles.front().y = {0.5, 0.0, 0.5};
  input.profiles.front().z = {0.0, 0.0, 0.0, 1.0, 0.0, 0.5};
  gvmt::ProfileExecution execution{input};
  execution.sample(0);
  const auto state{execution.sample(1'500'000'000)};
  ASSERT_TRUE(state.command);
  EXPECT_DOUBLE_EQ(state.command->vx, 0.375);
  EXPECT_DOUBLE_EQ(state.command->ax, 0.25);
  EXPECT_DOUBLE_EQ(state.command->vy, 0.5);
  EXPECT_DOUBLE_EQ(state.command->ay, 0.0);
  EXPECT_DOUBLE_EQ(state.command->wz, 0.5);
  EXPECT_DOUBLE_EQ(state.command->awz, 0.0);
}

/** @test An exact interval endpoint belongs to the following profile, not the old hold. */
TEST(ProfileExecution, AdvancesAtAnExactProfileBoundary)
{
  gvmt::ProfileExecution execution{sequence()};
  execution.sample(0);
  const auto boundary{execution.sample(4'000'000'000)};
  ASSERT_TRUE(boundary.command);
  EXPECT_EQ(boundary.profile_index, 1U);
  EXPECT_DOUBLE_EQ(boundary.command->vx, 0.5);
  EXPECT_DOUBLE_EQ(boundary.command->ax, -0.25);
}

/** @test A delayed callback skips expired intervals without producing a command burst. */
TEST(ProfileExecution, SkipsMultipleProfilesUsingElapsedTime)
{
  gvmt::ProfileExecution execution{sequence()};
  execution.sample(0);
  const auto current{execution.sample(6'500'000'000)};
  ASSERT_TRUE(current.command);
  EXPECT_EQ(current.profile_index, 2U);
  EXPECT_DOUBLE_EQ(current.command->vx, 0.0);
}

/** @test Backwards samples preserve the origin, active index, and last accepted timestamp. */
TEST(ProfileExecution, DiscardsBackwardsSamplesUntilTheClockRecovers)
{
  gvmt::ProfileExecution execution{sequence()};
  constexpr std::int64_t origin{1'000'000'000};
  execution.sample(origin);
  const auto accepted{execution.sample(origin + 4'500'000'000)};
  ASSERT_TRUE(accepted.command);
  EXPECT_EQ(accepted.profile_index, 1U);
  const auto discarded{execution.sample(origin + 2'000'000'000)};
  EXPECT_EQ(discarded.status, gvmt::ExecutionStatus::SkippedBackwardsTime);
  EXPECT_FALSE(discarded.command);
  EXPECT_EQ(discarded.profile_index, 1U);
  const auto still_old{execution.sample(origin + 3'000'000'000)};
  EXPECT_EQ(still_old.status, gvmt::ExecutionStatus::SkippedBackwardsTime);
  EXPECT_FALSE(still_old.command);
  const auto recovered{execution.sample(origin + 4'500'000'000)};
  ASSERT_TRUE(recovered.command);
  EXPECT_DOUBLE_EQ(recovered.command->vx, accepted.command->vx);
  const auto resumed{execution.sample(origin + 5'000'000'000)};
  ASSERT_TRUE(resumed.command);
  EXPECT_DOUBLE_EQ(resumed.command->vx, 0.25);
}

/** @test Finishing a nonzero final velocity produces no extra zero or terminal command. */
TEST(ProfileExecution, FinishesWithoutPublishingOrRestarting)
{
  auto input{sequence()};
  input.profiles.resize(1);
  input.profiles.front().duration = 1.0;
  input.profiles.front().x = {0.0, 0.5, 0.5};
  gvmt::ProfileExecution execution{input};
  const auto active{execution.sample(0)};
  ASSERT_TRUE(active.command);
  EXPECT_DOUBLE_EQ(active.command->vx, 0.5);
  const auto finished{execution.sample(1'000'000'000)};
  EXPECT_EQ(finished.status, gvmt::ExecutionStatus::Finished);
  EXPECT_FALSE(finished.command);
  EXPECT_FALSE(execution.sample(2'000'000'000).command);
  EXPECT_FALSE(execution.sample(0).command);
}

/** @test Prepared data retain their values after the original vector is destroyed or edited. */
TEST(ProfileExecution, OwnsItsPreparedProfiles)
{
  auto input{sequence()};
  gvmt::ProfileExecution execution{input};
  input.profiles.clear();
  execution.sample(0);
  const auto state{execution.sample(1'000'000'000)};
  ASSERT_TRUE(state.command);
  EXPECT_DOUBLE_EQ(state.command->vx, 0.25);
}

/** @test Large forward jumps finish safely without signed subtraction overflow. */
TEST(ProfileExecution, HandlesTheFullSignedTimestampSpan)
{
  gvmt::ProfileExecution execution{sequence()};
  ASSERT_TRUE(execution.sample(std::numeric_limits<std::int64_t>::min()).command);
  const auto end{execution.sample(std::numeric_limits<std::int64_t>::max())};
  EXPECT_EQ(end.status, gvmt::ExecutionStatus::Finished);
  EXPECT_FALSE(end.command);
}

/** @test The cursor constructor rejects the whole sequence before accepting any timestamps. */
TEST(ProfileExecution, RejectsInvalidInputBeforeExecution)
{
  auto input{sequence()};
  input.profiles.back().x.v_init = 0.1;
  input.profiles.back().x.v_end = 0.1;
  EXPECT_THROW(gvmt::ProfileExecution{input}, std::invalid_argument);
}

/** @test Examples with zero final velocities emit repeated zeros before timer completion. */
TEST(ProfileExecution, ExamplesIncludeARealFinalZeroPublicationWindow)
{
  std::size_t checked{0};
  for(const auto& file: std::filesystem::directory_iterator(PROFILE_CONFIG_DIR))
  {
    const auto name{file.path().filename().string()};
    if(name.rfind("example_", 0) != 0 || file.path().extension() != ".yaml" || name == "example_executor_params.yaml")
    {
      continue;
    }
    const auto input{gvmt::load_profile_file(file.path().string())};
    const auto& last{input.profiles.back()};
    if(last.x.v_end != 0.0 || last.y.v_end != 0.0 || last.z.w_end != 0.0)
    {
      continue;
    }
    gvmt::ProfileExecution execution{input};
    std::size_t trailing_zero_samples{0};
    for(std::int64_t stamp{0};; stamp += 20'000'000)
    {
      const auto state{execution.sample(stamp)};
      if(state.status == gvmt::ExecutionStatus::Finished)
      {
        break;
      }
      ASSERT_TRUE(state.command) << name;
      const bool stopped{state.command->vx == 0.0 && state.command->vy == 0.0 && state.command->wz == 0.0};
      trailing_zero_samples = stopped ? trailing_zero_samples + 1 : 0;
    }
    EXPECT_GE(trailing_zero_samples, 2U) << name;
    ++checked;
  }
  EXPECT_GT(checked, 0U);
}
