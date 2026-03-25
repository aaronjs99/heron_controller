#include <gtest/gtest.h>
#include <heron_controller/force_compensator.h>
#include <heron_controller/heron_constants.h>

TEST(ForceCompensatorTest, SaturateThrustersClampsForwardAndReverseLimits) {
  EXPECT_DOUBLE_EQ(MAX_FWD_THRUST, ForceCompensator::saturate_thrusters(MAX_FWD_THRUST + 5.0));
  EXPECT_DOUBLE_EQ(-MAX_BCK_THRUST, ForceCompensator::saturate_thrusters(-(MAX_BCK_THRUST + 5.0)));
  EXPECT_DOUBLE_EQ(12.5, ForceCompensator::saturate_thrusters(12.5));
}

TEST(ForceCompensatorTest, CalculateMotorSettingUsesDirectionalScaling) {
  EXPECT_DOUBLE_EQ(0.0, ForceCompensator::calculate_motor_setting(0.0));
  EXPECT_DOUBLE_EQ(MAX_OUTPUT, ForceCompensator::calculate_motor_setting(MAX_FWD_THRUST));
  EXPECT_DOUBLE_EQ(-MAX_OUTPUT, ForceCompensator::calculate_motor_setting(-MAX_BCK_THRUST));
  EXPECT_DOUBLE_EQ(0.5, ForceCompensator::calculate_motor_setting(MAX_FWD_THRUST * 0.5));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
