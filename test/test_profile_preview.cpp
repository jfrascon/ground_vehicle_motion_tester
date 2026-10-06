#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "ground_vehicle_motion_tester/profile_preview.hpp"

namespace gvmt = ground_vehicle_motion_tester;

namespace
{
  /** @brief Create a straight ramp followed by constant speed within a two-second profile. */
  gvmt::ProfileSequence straight_sequence()
  {
    gvmt::ProfileSequence sequence{};
    sequence.limits.x = {10.0, 10.0};
    sequence.limits.y = {10.0, 10.0};
    sequence.limits.z = {0.0, 0.0, 10.0, 10.0};
    gvmt::Profile profile{};
    profile.duration = 2.0;
    profile.x = {1.0, 0.0, 1.0};
    sequence.profiles.push_back(profile);
    return sequence;
  }

  /**
   * @brief Find a required event sample without relying on its index in the regular grid.
   * @param preview Computed plot data.
   * @param time Event time that must appear in the result.
   * @return The matching sample.
   * @throws std::logic_error A test fixture event was not included.
   */
  const gvmt::PreviewSample& at_time(const gvmt::ProfilePreview& preview, double time)
  {
    const auto& samples{preview.samples()};
    const auto found{std::find_if(samples.begin(), samples.end(), [time](const gvmt::PreviewSample& sample) {
      return std::abs(sample.time - time) < 1e-12;
    })};
    if(found == samples.end())
    {
      throw std::logic_error{"Required event sample is missing"};
    }
    return *found;
  }
}  // namespace

/** @test The ideal distance includes acceleration and the subsequent final-speed hold. */
TEST(ProfilePreview, IntegratesRampThenHoldAndKeepsFinalVelocity)
{
  const gvmt::ProfilePreview preview{straight_sequence()};
  const auto& samples{preview.samples()};
  ASSERT_GT(samples.size(), 2U);
  EXPECT_DOUBLE_EQ(samples.front().time, 0.0);
  EXPECT_DOUBLE_EQ(samples.front().vx, 0.0);
  EXPECT_DOUBLE_EQ(samples.front().ax, 1.0);
  EXPECT_DOUBLE_EQ(at_time(preview, 1.0).vx, 1.0);
  EXPECT_DOUBLE_EQ(at_time(preview, 1.0).ax, 0.0);
  EXPECT_DOUBLE_EQ(samples.back().time, 2.0);
  EXPECT_DOUBLE_EQ(samples.back().vx, 1.0);
  EXPECT_NEAR(samples.back().x, 1.5, 1e-12);
  EXPECT_NEAR(samples.back().y, 0.0, 1e-12);
}

/** @test Every profile and ramp boundary appears even when it is off the regular plot grid. */
TEST(ProfilePreview, IncludesIndependentOffGridRampEventsAndFinalEndpoint)
{
  auto sequence{straight_sequence()};
  auto& profile{sequence.profiles.front()};
  profile.duration = 0.73;
  profile.x = {2.0, 0.0, 0.35};
  profile.y = {1.0, 0.0, 0.27};
  profile.z = {0.0, 0.0, 0.0, 0.4, 0.0, 0.112};
  const gvmt::ProfilePreview preview{sequence};
  EXPECT_DOUBLE_EQ(at_time(preview, 0.175).ax, 0.0);
  EXPECT_DOUBLE_EQ(at_time(preview, 0.27).ay, 0.0);
  EXPECT_DOUBLE_EQ(at_time(preview, 0.28).awz, 0.0);
  EXPECT_DOUBLE_EQ(at_time(preview, 0.73).vx, 0.35);
  EXPECT_NEAR(preview.samples().back().theta, (0.5 * 0.112 * 0.28) + (0.112 * 0.45), 1e-10);
}

/** @test A profile boundary uses the next acceleration while preserving velocity continuity. */
TEST(ProfilePreview, HandlesConsecutiveAccelerationChanges)
{
  auto sequence{straight_sequence()};
  gvmt::Profile second{};
  second.duration = 1.0;
  second.x = {-1.0, 1.0, 0.0};
  sequence.profiles.push_back(second);
  const gvmt::ProfilePreview preview{sequence};
  EXPECT_DOUBLE_EQ(at_time(preview, 2.0).vx, 1.0);
  EXPECT_DOUBLE_EQ(at_time(preview, 2.0).ax, -1.0);
  EXPECT_DOUBLE_EQ(at_time(preview, 3.0).vx, 0.0);
  EXPECT_DOUBLE_EQ(at_time(preview, 3.0).ax, 0.0);
  EXPECT_NEAR(preview.samples().back().x, 2.0, 1e-12);
}

/** @test Decimal event times and regular grid times share one nanosecond clock entry. */
TEST(ProfilePreview, DoesNotDuplicateDecimalGridEvents)
{
  auto sequence{straight_sequence()};
  sequence.profiles.front().x = {1.0, 0.0, 0.3};
  const gvmt::ProfilePreview preview{sequence};
  const auto count{
    std::count_if(preview.samples().begin(), preview.samples().end(), [](const gvmt::PreviewSample& sample) {
      return std::abs(sample.time - 0.3) < 1e-12;
    })};
  EXPECT_EQ(count, 1);
}

/** @test Constant body-frame forward and yaw velocities produce the analytical circular arc. */
TEST(ProfilePreview, ReusesOdometryForConstantTwistCircularMotion)
{
  auto sequence{straight_sequence()};
  sequence.profiles.front().x = {0.0, 1.0, 1.0};
  sequence.profiles.front().z = {0.0, 0.0, 0.0, 0.0, 1.0, 1.0};
  const gvmt::ProfilePreview preview{sequence};
  const auto& final{preview.samples().back()};
  EXPECT_NEAR(final.x, std::sin(2.0), 1e-12);
  EXPECT_NEAR(final.y, 1.0 - std::cos(2.0), 1e-12);
  EXPECT_NEAR(final.theta, 2.0, 1e-12);
}

/**
 * @test Lateral translation keeps the platform heading even though its travel direction differs.
 */
TEST(ProfilePreview, IntegratesOmnidirectionalLateralMotion)
{
  auto sequence{straight_sequence()};
  sequence.profiles.front().x = {};
  sequence.profiles.front().y = {0.0, 1.0, 1.0};
  const gvmt::ProfilePreview preview{sequence};
  EXPECT_NEAR(preview.samples().back().x, 0.0, 1e-12);
  EXPECT_NEAR(preview.samples().back().y, 2.0, 1e-12);
  EXPECT_NEAR(preview.samples().back().theta, 0.0, 1e-12);
}

/** @test Changing the displayed-point density does not change the integration resolution. */
TEST(ProfilePreview, SeparatesPlotDensityFromOdometrySteps)
{
  const auto sequence{straight_sequence()};
  const gvmt::ProfilePreview dense{sequence, 0.1};
  const gvmt::ProfilePreview sparse{sequence, 0.5};
  EXPECT_GT(dense.samples().size(), sparse.samples().size());
  EXPECT_NEAR(dense.samples().back().x, sparse.samples().back().x, 1e-12);
}

/** @test Invalid input is rejected by the preview constructor before integration begins. */
TEST(ProfilePreview, ValidatesTheEntireSequenceInItsConstructor)
{
  auto sequence{straight_sequence()};
  gvmt::Profile second{};
  second.duration = 1.0;
  sequence.profiles.push_back(second);
  EXPECT_THROW(gvmt::ProfilePreview{sequence}, std::invalid_argument);
}

/** @test Plot spacing must be positive, finite, and representable in the integration clock. */
TEST(ProfilePreview, RejectsInvalidSamplePeriods)
{
  for(const auto period:
      {0.0, -0.1, 1e-12, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
  {
    EXPECT_THROW((gvmt::ProfilePreview{straight_sequence(), period}), std::invalid_argument);
  }
}

/** @test Valid mathematical profiles still report unsupported clock resolution or time overflow. */
TEST(ProfilePreview, ReportsUnrepresentablePreviewTimes)
{
  auto sequence{straight_sequence()};
  sequence.profiles.front().duration = 1e20;
  EXPECT_THROW(gvmt::ProfilePreview{sequence}, std::invalid_argument);
  sequence.profiles.front().duration = 1e-12;
  sequence.profiles.front().x = {};
  EXPECT_THROW(gvmt::ProfilePreview{sequence}, std::invalid_argument);
}
