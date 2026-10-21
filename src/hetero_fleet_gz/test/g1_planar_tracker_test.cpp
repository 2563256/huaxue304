#include "../src/g1_planar_tracker.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <limits>
using hetero_fleet::G1PlanarTracker;
void require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}
int main() {
  G1PlanarTracker tracker;
  tracker.update(0., 0., 0., .03, 0., .02);
  auto cmd = tracker.update(.20, .10, 0., .03, 0., .02);
  require(cmd[0] < 0. && cmd[1] < 0., "Low-speed overshoot must brake and correct sideways drift");
  require(cmd[0] >= -.15 && cmd[1] >= -.15, "Correction exceeded the gait velocity envelope");
  G1PlanarTracker rotated;
  const double yaw = std::acos(-1.) / 2;
  rotated.update(0., 0., yaw, .03, 0., .02);
  cmd = rotated.update(0., .20, yaw, .03, 0., .02);
  require(cmd[0] < 0. && std::abs(cmd[1]) < 1e-6, "World error must be transformed into the body frame");
  cmd = rotated.update(0., .20, yaw, 0., 0., .02);
  require(rotated.holding() && std::hypot(cmd[0],cmd[1]) < 1e-9, "Stopping must anchor the actual position");
  cmd = rotated.update(0., .22, yaw, 0., 0., .02);
  require(cmd[0] < 0., "Stationary hold lost its position correction");
  cmd = rotated.update(10., 10., yaw, .03, 0., -1.);
  require(std::hypot(cmd[0],cmd[1]) < .05, "Clock reset retained stale position error");
  G1PlanarTracker blocked;
  for (int i=0;i<10000;++i) {
    cmd = blocked.update(0.,0.,0.,.40,0.,.02);
    require(cmd[0] <= .40 && std::abs(cmd[1]) <= .15, "Blocked command exceeded velocity limits");
  }
  cmd = blocked.update(0.,0.,0.,0.,0.,.02);
  require(std::hypot(cmd[0],cmd[1]) < 1e-9, "Stopping replayed accumulated walking demand");
  cmd = blocked.update(std::numeric_limits<double>::quiet_NaN(),0.,0.,.1,0.,.02);
  require(cmd[0]==0. && cmd[1]==0., "Invalid pose must fail closed");
  std::cout << "PASS: low-speed drift, body transform, hold, reset, bounded demand and invalid pose\n";
}
