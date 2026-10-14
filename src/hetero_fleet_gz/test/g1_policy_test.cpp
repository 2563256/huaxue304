#include "../src/g1_policy.hpp"
#include <iostream>
int main(int argc, char **argv) {
  if (argc != 3)
    return 2;
  hetero_fleet::G1Policy policy(argv[1]);
  std::ifstream reference(argv[2]);
  Eigen::VectorXf first;
  float worst = 0;
  for (int t = 0; t < 100; ++t) {
    Eigen::VectorXf observation(47);
    for (int i = 0; i < 47; ++i)
      observation[i] = std::sin(float(t * 47 + i) * .1f);
    auto actual = policy.infer(observation);
    if (t == 0)
      first = actual;
    for (int i = 0; i < 12; ++i) {
      float expected;
      if (!(reference >> expected))
        return 3;
      worst = std::max(worst, std::abs(actual[i] - expected));
    }
  }
  policy.reset();
  Eigen::VectorXf observation(47);
  for (int i = 0; i < 47; ++i)
    observation[i] = std::sin(float(i) * .1f);
  if ((policy.infer(observation) - first).norm() > 1e-6)
    return 4;
  std::cout << "100-step native/Torch recurrence error: " << worst << '\n';
  return worst < 1e-4 ? 0 : 1;
}
