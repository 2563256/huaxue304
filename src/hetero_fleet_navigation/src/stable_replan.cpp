#include <behaviortree_cpp/bt_factory.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <optional>

// Children: path validity check, then path computation. Holding the old route
// keeps FollowPath active; its local obstacles and final collision stop remain.
class StableReplan : public BT::ControlNode {
public:
 StableReplan(const std::string& name,const BT::NodeConfig& config)
 : BT::ControlNode(name,config) {node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");}
 static BT::PortsList providedPorts() {
  return {BT::InputPort<nav_msgs::msg::Path>("path"),
          BT::InputPort<double>("blocked_duration",5.,"Continuous obstruction seconds in ROS time"),
          BT::InputPort<double>("min_interval",15.,"Minimum path replacement interval in ROS time")};
 }
 BT::NodeStatus tick() override {
  if(children_nodes_.size()!=2)throw BT::RuntimeError("StableReplan needs validity and computation children");
  if(children_nodes_[1]->status()==BT::NodeStatus::RUNNING)return compute();
  auto validity=children_nodes_[0]->executeTick();haltChild(0);
  if(validity==BT::NodeStatus::SUCCESS){blocked_since_.reset();return BT::NodeStatus::SUCCESS;}
  const auto now=node_->now();nav_msgs::msg::Path path;getInput("path",path);
  if(path.poses.empty())return compute();
  double blocked=5.,interval=15.;getInput("blocked_duration",blocked);getInput("min_interval",interval);
  const int64_t stamp=static_cast<int64_t>(path.header.stamp.sec)*1000000000LL+path.header.stamp.nanosec;
  if(!blocked_since_ || now.nanoseconds()<*blocked_since_ || stamp!=path_stamp_)blocked_since_=now.nanoseconds();
  path_stamp_=stamp;
  const double blocked_age=(now.nanoseconds()-*blocked_since_)*1e-9;
  const double path_age=(now.nanoseconds()-stamp)*1e-9;
  if(blocked_age<blocked || (path_age>=0. && path_age<interval))return BT::NodeStatus::SUCCESS;
  return compute();
 }
private:
 BT::NodeStatus compute() {
  auto status=children_nodes_[1]->executeTick();
  if(status!=BT::NodeStatus::RUNNING){haltChild(1);blocked_since_.reset();}
  return status;
 }
 rclcpp::Node::SharedPtr node_;
 std::optional<int64_t> blocked_since_;
 int64_t path_stamp_=0;
};
BT_REGISTER_NODES(factory){factory.registerNodeType<StableReplan>("StableReplan");}
