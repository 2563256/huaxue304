#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace hetero_fleet {
// Close the planar position loop around the learned gait, including small
// nonzero walking commands. The output is still a policy velocity request;
// only the existing physical joint torques can move the robot.
class G1PlanarTracker {
public:
  bool holding() const { return hold_; }
  std::array<double, 2> update(double x, double y, double yaw,
                               double vx, double vy, double dt) {
    for (double v : {x, y, yaw, vx, vy, dt}) {
      if (!std::isfinite(v)) { initialized_ = false; return {0., 0.}; }
    }
    const bool stationary = std::hypot(vx, vy) < .01;
    if (!initialized_ || dt <= 0. || dt > .1 || stationary != hold_) {
      rx_ = x; ry_ = y; initialized_ = true;
    }
    hold_ = stationary;
    const double c = std::cos(yaw), s = std::sin(yaw);
    dt = std::clamp(dt, 0., .1);
    if (!stationary) {
      rx_ += (c * vx - s * vy) * dt;
      ry_ += (s * vx + c * vy) * dt;
    }
    double ex = rx_ - x, ey = ry_ - y;
    // Bound accumulated position demand when a wall/peer prevents tracking.
    const double error = std::hypot(ex, ey);
    if (error > .25) {
      ex *= .25 / error; ey *= .25 / error;
      rx_ = x + ex; ry_ = y + ey;
    }
    return {std::clamp((stationary ? 0. : vx) + 1.2 * (c * ex + s * ey), -.15, .40),
            std::clamp((stationary ? 0. : vy) + 1.2 * (-s * ex + c * ey), -.15, .15)};
  }
private:
  bool initialized_ = false, hold_ = false;
  double rx_ = 0., ry_ = 0.;
};
}  // namespace hetero_fleet
