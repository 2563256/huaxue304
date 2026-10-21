#include "g1_policy.hpp"
#include "g1_planar_tracker.hpp"
#include <algorithm>
#include <array>
#include <geometry_msgs/msg/twist.hpp>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
namespace hetero_fleet {
class G1Locomotion final : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate {
  std::vector<gz::sim::Joint> joints;
  std::vector<std::string> names;
  gz::sim::Link base;
  gz::sim::Entity entity;
  std::unique_ptr<G1Policy> policy;
  std::shared_ptr<rclcpp::Context> context;
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr states;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status;
  std::array<double, 12> nominal{-.1, 0, 0, .3, -.2, 0, -.1, 0, 0, .3, -.2, 0};
  std::array<double, 12> kp{100, 100, 100, 150, 40, 40,
                            100, 100, 100, 150, 40, 40};
  std::array<double, 12> kd{2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2};
  std::array<double, 12> limit{88, 139, 88, 139, 50, 50,
                               88, 139, 88, 139, 50, 50};
  Eigen::VectorXf action = Eigen::VectorXf::Zero(12);
  std::array<double, 12> target = nominal;
  gz::math::Vector3d command;
  double scale = 1, now = 0, start = -1, nextPolicy = 0, lastCmd = -100,
         lastPub = -100;
  bool initialized = false, fallen = false;
  G1PlanarTracker planarTracker;
  double heading = 0, lastControl = 0;
  void Configure(const gz::sim::Entity &e,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    entity = e;
    scale = sdf->Get<double>("scale");
    auto name = sdf->Get<std::string>("robot_name");
    policy = std::make_unique<G1Policy>(sdf->Get<std::string>("policy_path"));
    gz::sim::Model model(e);
    base = gz::sim::Link(model.CanonicalLink(ecm));
    base.EnableVelocityChecks(ecm);
    for (auto side : {"left", "right"})
      for (auto joint : {"hip_pitch", "hip_roll", "hip_yaw", "knee",
                         "ankle_pitch", "ankle_roll"}) {
        names.push_back(std::string(side) + "_" + joint + "_joint");
        auto id = model.JointByName(ecm, names.back());
        if (id == gz::sim::kNullEntity)
          throw std::runtime_error("Missing G1 joint " + names.back());
        joints.emplace_back(id);
        joints.back().EnablePositionCheck(ecm);
        joints.back().EnableVelocityCheck(ecm);
      }
    context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context);
    node = std::make_shared<rclcpp::Node>("locomotion_controller", "/" + name,
                                          options);
    rclcpp::ExecutorOptions eo;
    eo.context = context;
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
    executor->add_node(node);
    sub = node->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          command.Set(std::clamp(msg->linear.x, -.15, .40),
                      std::clamp(msg->linear.y, -.15, .15),
                      std::clamp(msg->angular.z, -.65, .65));
          lastCmd = now;
        });
    states = node->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);
    health = node->create_publisher<std_msgs::msg::Bool>(
        "locomotion_ready", rclcpp::QoS(1).transient_local());
    status =
        node->create_publisher<std_msgs::msg::String>("locomotion_status", 10);
    RCLCPP_INFO(node->get_logger(),
                "G1 native LSTM + physical joint torque control, scale %.2f",
                scale);
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused)
      return;
    now = std::chrono::duration<double>(info.simTime).count();
    if (start < 0)
      start = now;
    if (!initialized) {
      for (int i = 0; i < 12; ++i)
        joints[i].ResetPosition(ecm, {nominal[i]});
      initialized = true;
    }
    std::array<double, 12> q{}, dq{};
    for (int i = 0; i < 12; ++i) {
      auto a = joints[i].Position(ecm), b = joints[i].Velocity(ecm);
      if (!a || !b || a->empty() || b->empty())
        return;
      q[i] = (*a)[0];
      dq[i] = (*b)[0];
    }
    auto pose = gz::sim::worldPose(base.Entity(), ecm);
    auto omega =
        base.WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);
    auto rot = pose.Rot();
    auto gravity = rot.RotateVectorReverse({0, 0, -1});
    // A fall is latched; never slide/teleport a fallen humanoid to finish a
    // task.
    if (now - start > 1.0 && gravity.Z() > -.65 && !fallen) {
      fallen = true;
      RCLCPP_ERROR(node->get_logger(),
                   "G1 fell; locomotion disabled, restart simulation to reset");
    }
    if (now >= nextPolicy) {
      nextPolicy = now + .02 * std::sqrt(scale);
      executor->spin_some();
      auto cmd = (now - lastCmd < .5 && now - start > 2 && !fallen)
                     ? command
                     : gz::math::Vector3d::Zero;
      double dt = lastControl > 0 ? now - lastControl : .02 * std::sqrt(scale);
      lastControl = now;
      if (now - start < .1)
        heading = rot.Yaw();
      const bool wasHolding = planarTracker.holding();
      auto planar = planarTracker.update(pose.Pos().X(), pose.Pos().Y(), rot.Yaw(),
                                          cmd.X(), cmd.Y(), dt);
      if (planarTracker.holding() && !wasHolding)
        heading = rot.Yaw();
      cmd.X(planar[0]);
      cmd.Y(planar[1]);
      heading += cmd.Z() * dt;
      auto headingError = std::atan2(std::sin(heading - rot.Yaw()),
                                     std::cos(heading - rot.Yaw()));
      cmd.Z(std::clamp(cmd.Z() + 1.5 * headingError, -.8, .8));
      auto localOmega = rot.RotateVectorReverse(omega) * std::sqrt(scale);
      Eigen::VectorXf obs = Eigen::VectorXf::Zero(47);
      obs.head(9) << localOmega.X() * .25, localOmega.Y() * .25,
          localOmega.Z() * .25, gravity.X(), gravity.Y(), gravity.Z(),
          cmd.X() * 2 / std::sqrt(scale), cmd.Y() * 2 / std::sqrt(scale),
          cmd.Z() * .25 * std::sqrt(scale);
      for (int i = 0; i < 12; ++i) {
        obs[9 + i] = q[i] - nominal[i];
        obs[21 + i] = dq[i] * .05 * std::sqrt(scale);
        obs[33 + i] = action[i];
      }
      auto phase = std::fmod((now - start) / std::sqrt(scale), .8) / .8;
      obs[45] = std::sin(2 * M_PI * phase);
      obs[46] = std::cos(2 * M_PI * phase);
      if (now - start > .1 && !fallen) {
        action = policy->infer(obs);
        for (int i = 0; i < 12; ++i)
          target[i] = nominal[i] + .25 * std::clamp(double(action[i]), -8., 8.);
      }
    }
    for (int i = 0; i < 12; ++i) {
      double torque = fallen ? -kd[i] * dq[i]
                             : kp[i] * std::pow(scale, 4) * (target[i] - q[i]) -
                                   kd[i] * std::pow(scale, 4.5) * dq[i];
      joints[i].SetForce(ecm,
                         {std::clamp(torque, -limit[i] * std::pow(scale, 4),
                                     limit[i] * std::pow(scale, 4))});
    }
    if (now - lastPub > .05) {
      lastPub = now;
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = rclcpp::Time(int64_t(now * 1e9));
      msg.name = names;
      msg.position.assign(q.begin(), q.end());
      msg.velocity.assign(dq.begin(), dq.end());
      states->publish(msg);
      std_msgs::msg::Bool ok;
      ok.data = !fallen && now - start > 2;
      health->publish(ok);
      std_msgs::msg::String text;
      text.data = fallen ? "fallen" : (ok.data ? "ready" : "initializing");
      status->publish(text);
    }
  }
};
} // namespace hetero_fleet
GZ_ADD_PLUGIN(hetero_fleet::G1Locomotion, gz::sim::System,
              hetero_fleet::G1Locomotion::ISystemConfigure,
              hetero_fleet::G1Locomotion::ISystemPreUpdate)
