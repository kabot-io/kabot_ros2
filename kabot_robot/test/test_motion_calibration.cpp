#include <kabot_robot/motion_calibration.hpp>
#include <cassert>
#include <limits>

int main()
{
  const kabot_robot::MotionCalibration calibration;
  const kabot_robot::MotionModel model(calibration);
  const auto near = [](double actual, double expected) { assert(std::abs(actual - expected) < 1e-9); };
  for (const auto speed : {calibration.low_speed, calibration.high_speed}) {
    const auto forward = model.map(speed, 0);
    const auto expected = speed == calibration.low_speed ? calibration.low_effort : calibration.high_effort;
    near(forward.linear, speed);
    near(forward.angular, 0);
    near(forward.effort[0], expected[0]);
    near(forward.effort[1], expected[1]);
    const auto reverse = model.map(-speed, 0);
    near(reverse.effort[0], -expected[0]);
    near(reverse.effort[1], -expected[1]);
  }
  for (const auto angular : {calibration.ccw_rate, -calibration.cw_rate}) {
    const auto turn = model.map(0, angular);
    near(turn.angular, angular);
    near(turn.linear, 0);
    near(turn.effort[0], angular > 0 ? -1 : 1);
    near(turn.effort[1], angular > 0 ? 1 : -1);
  }
  for (const double linear : {-0.2, -0.02, 0.0, 0.02, 0.2}) {
    for (const double angular : {-3.0, -0.3, 0.0, 0.3, 3.0}) {
      const auto output = model.map(linear, angular);
      assert(output.scale > 0 && output.scale <= 1);
      assert(std::abs(output.effort[0]) <= 1 && std::abs(output.effort[1]) <= 1);
      near(output.linear, linear * output.scale);
      near(output.angular, angular * output.scale);
    }
  }
  for (const auto invalid : {0.0, std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::infinity()}) {
    const auto stopped = model.map(invalid, 0);
    near(stopped.linear, 0);
    near(stopped.angular, 0);
    near(stopped.effort[0], 0);
    near(stopped.effort[1], 0);
  }
  auto bad = calibration;
  bad.low_speed = bad.high_speed;
  bool rejected = false;
  try { const kabot_robot::MotionModel invalid(bad); }
  catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
}