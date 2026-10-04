#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace kabot_robot
{
struct MotionCalibration
{
  double low_speed = 0.575 / 17.05;
  double high_speed = 1.265 / 18.59;
  std::array<double, 2> low_effort{0.75478125, 0.74521875};
  std::array<double, 2> high_effort{1.0, 0.9611125};
  double ccw_rate = 10.0 * std::acos(-1.0) / 22.9;
  double cw_rate = 10.0 * std::acos(-1.0) / 22.93;

  void validate() const
  {
    if (!std::isfinite(low_speed) || !std::isfinite(high_speed) ||
        !std::isfinite(ccw_rate) || !std::isfinite(cw_rate) ||
        low_speed <= 0 || high_speed <= low_speed || ccw_rate <= 0 || cw_rate <= 0) {
      throw std::invalid_argument("Invalid calibration speeds");
    }
    for (std::size_t wheel = 0; wheel < 2; ++wheel) {
      const double slope = (high_effort[wheel] - low_effort[wheel]) / (high_speed - low_speed);
      const double offset = low_effort[wheel] - slope * low_speed;
      if (!std::isfinite(low_effort[wheel]) || !std::isfinite(high_effort[wheel]) ||
          low_effort[wheel] <= 0 || high_effort[wheel] > 1 || slope <= 0 ||
          offset < 0 || offset >= 1) {
        throw std::invalid_argument("Invalid calibration efforts");
      }
    }
  }
};

struct MotionOutput
{
  std::array<double, 2> effort{};
  double linear = 0;
  double angular = 0;
  double scale = 1;
};

class MotionModel
{
public:
  explicit MotionModel(const MotionCalibration & calibration = {}) : calibration_(calibration)
  {
    calibration_.validate();
    for (std::size_t wheel = 0; wheel < 2; ++wheel) {
      slope_[wheel] = (calibration_.high_effort[wheel] - calibration_.low_effort[wheel]) /
        (calibration_.high_speed - calibration_.low_speed);
      offset_[wheel] = calibration_.low_effort[wheel] - slope_[wheel] * calibration_.low_speed;
      maximum_speed_[wheel] = (1.0 - offset_[wheel]) / slope_[wheel];
    }
  }

  MotionOutput map(double linear, double angular) const
  {
    if (!std::isfinite(linear) || !std::isfinite(angular)) {
      return {};
    }
    const double turn_rate = angular >= 0 ? calibration_.ccw_rate : calibration_.cw_rate;
    std::array<double, 2> demand{
      linear - angular * maximum_speed_[0] / turn_rate,
      linear + angular * maximum_speed_[1] / turn_rate};
    if (!std::isfinite(demand[0]) || !std::isfinite(demand[1])) {
      return {};
    }
    const double ratio = std::max({1.0, std::abs(demand[0]) / maximum_speed_[0],
                                  std::abs(demand[1]) / maximum_speed_[1]});
    MotionOutput output;
    output.scale = 1.0 / ratio;
    output.linear = linear * output.scale;
    output.angular = angular * output.scale;
    for (std::size_t wheel = 0; wheel < 2; ++wheel) {
      const double speed = demand[wheel] * output.scale;
      output.effort[wheel] = std::abs(speed) < 1e-12 ? 0.0 :
        std::copysign(std::min(1.0, offset_[wheel] + slope_[wheel] * std::abs(speed)), speed);
    }
    return output;
  }

private:
  MotionCalibration calibration_;
  std::array<double, 2> slope_{};
  std::array<double, 2> offset_{};
  std::array<double, 2> maximum_speed_{};
};
}