#include <behaviortree_cpp/bt_factory.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <stdexcept>

class AsyncValidity : public BT::StatefulActionNode {
public:
 using BT::StatefulActionNode::StatefulActionNode;
 static BT::PortsList providedPorts() {return {};}
 static inline int replies = 0;
 BT::NodeStatus onStart() override {return BT::NodeStatus::RUNNING;}
 BT::NodeStatus onRunning() override {++replies; return BT::NodeStatus::SUCCESS;}
 void onHalted() override {}
};
int main(int argc, char **argv) {
 if (argc < 3) return 2;
 rclcpp::init(argc, argv);
 auto node = std::make_shared<rclcpp::Node>("goal_route_regression");
 BT::BehaviorTreeFactory factory;
 factory.registerFromPlugin(argv[1]);
 factory.registerNodeType<AsyncValidity>("AsyncValidity");
 bool can_plan = true;
 int plans = 0;
 auto board = BT::Blackboard::create();
 board->set("node", node);
 geometry_msgs::msg::PoseStamped goal;
 goal.header.frame_id = "map"; goal.pose.orientation.w = 1.; goal.pose.position.x = 1.;
 board->set("goal", goal); board->set("path", nav_msgs::msg::Path{});
 factory.registerSimpleCondition("Valid", [](BT::TreeNode &) {return BT::NodeStatus::SUCCESS;});
 factory.registerSimpleAction("Plan", [&](BT::TreeNode &n) {
  ++plans;
  if (!can_plan) return BT::NodeStatus::FAILURE;
  nav_msgs::msg::Path path; path.header.frame_id = "map"; path.header.stamp = node->now();
  path.poses = {board->get<geometry_msgs::msg::PoseStamped>("goal")};
  n.setOutput("path", path);
  return BT::NodeStatus::SUCCESS;
 }, {BT::OutputPort<nav_msgs::msg::Path>("path")});
 auto tree = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Test"><StableReplan goal="{goal}" path="{path}"><Valid/><Plan path="{path}"/></StableReplan></BehaviorTree></root>)", board);
 auto require = [](bool value, const char *message) {if (!value) throw std::runtime_error(message);};
 require(tree.tickOnce() == BT::NodeStatus::SUCCESS && plans == 1, "Initial goal must compute a route");
 tree.tickOnce(); require(plans == 1, "Unchanged valid route should be cached");
 goal.pose.position.x = 2.; board->set("goal", goal); can_plan = false;
 require(tree.tickOnce() == BT::NodeStatus::FAILURE, "Failed new goal reused old route");
 require(board->get<nav_msgs::msg::Path>("path").poses.empty(), "Old-goal path survived planning failure");
 require(tree.tickOnce() == BT::NodeStatus::FAILURE && plans == 3, "Failed goal must retry even if validity says success");
 can_plan = true;
 require(tree.tickOnce() == BT::NodeStatus::SUCCESS, "Goal retry did not recover");
 require(board->get<nav_msgs::msg::Path>("path").poses.back().pose.position.x == 2., "Committed wrong target");
 board->set("path", nav_msgs::msg::Path{});
 tree.tickOnce(); require(plans == 5, "Empty path was treated as a valid route");
 auto async_tree = factory.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Test"><StableReplan goal="{goal}" path="{path}"><AsyncValidity/><Plan path="{path}"/></StableReplan></BehaviorTree></root>)", board);
 async_tree.tickOnce();
 require(async_tree.tickOnce() == BT::NodeStatus::RUNNING, "Expected in-flight validity request");
 require(async_tree.tickOnce() == BT::NodeStatus::SUCCESS && AsyncValidity::replies == 1,
         "Validity request was halted before response");
 const int before_resume = plans;
 goal = board->get<geometry_msgs::msg::PoseStamped>("goal");
 goal.header.stamp = node->now(); board->set("goal", goal);
 require(async_tree.tickOnce() == BT::NodeStatus::SUCCESS && plans == before_resume + 1,
         "A resumed same-target action reused a route from before retreat");
 // Use Nav2's real RecoveryNode: a transient occupied goal waits and retries,
 // whereas an invalid goal outside the map must fail without retry motion.
 factory.registerFromPlugin(argv[2]);
 int attempts = 0, pauses = 0;
 uint16_t failure_code = 206;
 factory.registerSimpleAction("TryGoal", [&](BT::TreeNode &) {
  ++attempts; board->set("compute_path_error_code", failure_code);
  return attempts == 1 ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
 });
 factory.registerSimpleAction("PauseForPeer", [&](BT::TreeNode &) {
  ++pauses; return BT::NodeStatus::SUCCESS;
 });
 factory.registerSimpleCondition("Unrecoverable", [](BT::TreeNode &) {return BT::NodeStatus::FAILURE;});
 const std::string recovery_xml = R"(<root BTCPP_format="4"><BehaviorTree ID="Test"><RecoveryNode number_of_retries="6"><TryGoal/><Fallback><Sequence><ScriptCondition code="compute_path_error_code == 206"/><PauseForPeer/></Sequence><Unrecoverable/></Fallback></RecoveryNode></BehaviorTree></root>)";
 auto recovery = factory.createTreeFromText(recovery_xml, board);
 BT::NodeStatus status;
 do {status = recovery.tickOnce();} while (status == BT::NodeStatus::RUNNING);
 require(status == BT::NodeStatus::SUCCESS && attempts == 2 && pauses == 1,
         "Temporary goal occupancy was not retried after waiting");
 attempts = pauses = 0; failure_code = 204;
 auto invalid = factory.createTreeFromText(recovery_xml, board);
 require(invalid.tickOnce() == BT::NodeStatus::FAILURE && attempts == 1 && pauses == 0,
         "Invalid outside-map goal should not wait or retry");
 std::cout << "PASS: goal-bound routes, failed goal retries, empty path, async validity response\n";
 rclcpp::shutdown();
}
