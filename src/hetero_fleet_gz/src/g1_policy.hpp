#pragma once
#include <Eigen/Core>
#include <cmath>
#include <fstream>
#include <stdexcept>
namespace hetero_fleet {
// Unitree's 47-input, 64-state LSTM followed by 64->32 ELU->12 actor.
// Native inference avoids a Python/Torch dependency in the Gazebo update loop.
class G1Policy {
  using Matrix =
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  Matrix wi{256, 47}, wh{256, 64}, w0{32, 64}, w2{12, 32};
  Eigen::VectorXf bi{256}, bh{256}, b0{32}, b2{12},
      h = Eigen::VectorXf::Zero(64), c = h;
  template <class T> void read(std::ifstream &f, T &v) {
    f.read(reinterpret_cast<char *>(v.data()), v.size() * sizeof(float));
    if (!f)
      throw std::runtime_error("Invalid G1 policy weights");
  }

public:
  explicit G1Policy(const std::string &path) {
    std::ifstream f(path, std::ios::binary);
    char magic[8]{};
    f.read(magic, 8);
    if (std::string(magic, 8) != "G1LSTM01")
      throw std::runtime_error("G1 policy file missing or invalid: " + path);
    read(f, wi);
    read(f, wh);
    read(f, bi);
    read(f, bh);
    read(f, w0);
    read(f, b0);
    read(f, w2);
    read(f, b2);
  }
  void reset() {
    h.setZero();
    c.setZero();
  }
  Eigen::VectorXf infer(const Eigen::VectorXf &obs) {
    Eigen::VectorXf g = wi * obs + wh * h + bi + bh;
    for (int k = 0; k < 64; ++k) {
      float i = 1 / (1 + std::exp(-g[k])), f = 1 / (1 + std::exp(-g[64 + k])),
            o = 1 / (1 + std::exp(-g[192 + k]));
      c[k] = f * c[k] + i * std::tanh(g[128 + k]);
      h[k] = o * std::tanh(c[k]);
    }
    Eigen::VectorXf a = w0 * h + b0;
    for (int k = 0; k < a.size(); ++k)
      if (a[k] < 0)
        a[k] = std::expm1(a[k]);
    return w2 * a + b2;
  }
};
} // namespace hetero_fleet
