#include <behaviortree_cpp/bt_factory.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <optional>

// Children: asynchronous path validity check, then path computation. A route
// may be retained through transient traffic only for the SAME navigation goal.
class StableReplan : public BT::ControlNode {
public:
 StableReplan(const std::string &name, const BT::NodeConfig &config)
 : BT::ControlNode(name, config) {
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_->has_parameter("mapf_rolling_period_sec"))
   node_->declare_parameter("mapf_rolling_period_sec", 0.0);
 }
 static BT::PortsList providedPorts() {
  return {BT::BidirectionalPort<nav_msgs::msg::Path>("path"),
          BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
          BT::InputPort<double>("blocked_duration", 5., "Continuous obstruction seconds in ROS time"),
          BT::InputPort<double>("min_interval", 15., "Minimum path replacement interval in ROS time")};
 }
 BT::NodeStatus tick() override {
  if (children_nodes_.size() != 2) throw BT::RuntimeError("StableReplan needs validity and computation children");
  geometry_msgs::msg::PoseStamped desired;
  const bool goal_known = bool(getInput("goal", desired));
  if (goal_known && (!goal_ || !sameGoal(*goal_, desired))) {
   haltChildren();
   goal_ = desired;
   committed_ = false;
   retry_ = true;
   blocked_since_.reset();
   clearPath();
  }
  if (children_nodes_[1]->status() == BT::NodeStatus::RUNNING) return compute();
  nav_msgs::msg::Path path;
  getInput("path", path);
  // Some validity services accept an empty path. It never authorizes motion.
  if (path.poses.empty() || retry_ || (goal_known && !committed_)) return compute();
  const auto validity = children_nodes_[0]->executeTick();
  // Do not halt an outstanding service request before its response arrives.
  if (validity == BT::NodeStatus::RUNNING) return BT::NodeStatus::RUNNING;
  haltChild(0);
  // Copy B: periodically refresh the unexecuted suffix from current TF.
  // The executing action and physical safety gate stay in place throughout.
  const auto route_stamp = rclcpp::Time(path.header.stamp, node_->get_clock()->get_clock_type());
  if (validity == BT::NodeStatus::SUCCESS &&
      node_->has_parameter("mapf_rolling_period_sec") &&
      node_->get_parameter("mapf_rolling_period_sec").as_double() > 0. &&
      (node_->now()-route_stamp).seconds() >= node_->get_parameter("mapf_rolling_period_sec").as_double()) {
   retry_ = true;
   return compute();
  }
  if (validity == BT::NodeStatus::SUCCESS) {
   blocked_since_.reset();
   return BT::NodeStatus::SUCCESS;
  }
  const auto now = node_->now();
  double blocked = 5., interval = 15.;
  getInput("blocked_duration", blocked); getInput("min_interval", interval);
  const int64_t stamp = static_cast<int64_t>(path.header.stamp.sec) * 1000000000LL + path.header.stamp.nanosec;
  if (!blocked_since_ || now.nanoseconds() < *blocked_since_ || stamp != path_stamp_)
   blocked_since_ = now.nanoseconds();
  path_stamp_ = stamp;
  const double blocked_age = (now.nanoseconds() - *blocked_since_) * 1e-9;
  const double path_age = (now.nanoseconds() - stamp) * 1e-9;
  if (blocked_age < blocked || (path_age >= 0. && path_age < interval)) return BT::NodeStatus::SUCCESS;
  clearPath();
  retry_ = true;
  return compute();
 }
private:
 static bool sameGoal(const geometry_msgs::msg::PoseStamped &a, const geometry_msgs::msg::PoseStamped &b) {
  const auto &p = a.pose.position; const auto &q = b.pose.position;
  const auto &u = a.pose.orientation; const auto &v = b.pose.orientation;
  const double direct = std::abs(u.x-v.x)+std::abs(u.y-v.y)+std::abs(u.z-v.z)+std::abs(u.w-v.w);
  const double negated = std::abs(u.x+v.x)+std::abs(u.y+v.y)+std::abs(u.z+v.z)+std::abs(u.w+v.w);
  // Resuming after a queue retreat is a new action even at the same target.
  // Its route must start at the current body pose, not the previous action's.
  return a.header.frame_id == b.header.frame_id && a.header.stamp == b.header.stamp &&
    std::hypot(p.x-q.x, p.y-q.y) < 1e-6 && std::abs(p.z-q.z) < 1e-6 &&
    std::min(direct, negated) < 1e-6;
 }
 void clearPath() {setOutput("path", nav_msgs::msg::Path{});}
 BT::NodeStatus compute() {
  const auto status = children_nodes_[1]->executeTick();
  if (status == BT::NodeStatus::RUNNING) return status;
  haltChild(1); blocked_since_.reset();
  nav_msgs::msg::Path path; getInput("path", path);
  if (status == BT::NodeStatus::SUCCESS && !path.poses.empty()) {
   retry_ = false; committed_ = true;
   return BT::NodeStatus::SUCCESS;
  }
  retry_ = true; committed_ = false; clearPath();
  return BT::NodeStatus::FAILURE;
 }
 rclcpp::Node::SharedPtr node_;
 std::optional<int64_t> blocked_since_;
 std::optional<geometry_msgs::msg::PoseStamped> goal_;
 int64_t path_stamp_ = 0;
 bool committed_ = false, retry_ = false;
};
BT_REGISTER_NODES(factory) {factory.registerNodeType<StableReplan>("StableReplan");}
