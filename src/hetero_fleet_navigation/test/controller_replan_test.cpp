// Execute the production recovery subtree with the real Nav2 RecoveryNode.
#include <behaviortree_cpp/bt_factory.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

struct Fixture {
 int follow=0, plans=0, clears=0, compute_ticks=0;
 bool tracking_failed=false, planner_fails=false, recoverable=true;
 int fail_times=1;
};
int main(int argc,char **argv) {
 if(argc!=3)return 2;
 BT::BehaviorTreeFactory factory;factory.registerFromPlugin(argv[1]);
 Fixture state;
 factory.registerSimpleCondition("WouldAControllerRecoveryHelp",[&](BT::TreeNode&){
   return state.recoverable?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;
 },{BT::InputPort<int>("error_code")});
 factory.registerSimpleAction("ClearEntireCostmap",[&](BT::TreeNode&){
   ++state.clears;return BT::NodeStatus::SUCCESS;
 },{BT::InputPort<std::string>("service_name")});
 factory.registerSimpleAction("FollowPath",[&](BT::TreeNode& n){
   nav_msgs::msg::Path path;n.getInput("path",path);++state.follow;
   if(path.poses.empty())throw std::runtime_error("Empty path sent to controller");
   if(state.follow<=state.fail_times){state.tracking_failed=true;return BT::NodeStatus::FAILURE;}
   if(state.tracking_failed && path.poses.front().pose.position.x!=11.817)
     throw std::runtime_error("Controller retried the old collision-free but untrackable path");
   return BT::NodeStatus::SUCCESS;
 },{BT::InputPort<nav_msgs::msg::Path>("path"),BT::InputPort<std::string>("controller_id"),
    BT::InputPort<std::string>("goal_checker_id"),BT::OutputPort<int>("error_code_id")});
 factory.registerSimpleAction("ComputePathToPose",[&](BT::TreeNode& n){
   ++state.plans;
   geometry_msgs::msg::PoseStamped goal;n.getInput("goal",goal);
   if(goal.pose.position.x!=1.7)throw std::runtime_error("Recovery changed the task goal");
   std::string planner;n.getInput("planner_id",planner);
   if(planner!="GridBased" && planner!="TaskGridBased" && planner!="RetreatGridBased")
     throw std::runtime_error("Wrong recovery planner");
   nav_msgs::msg::Path path;
   if(state.planner_fails){n.setOutput("path",path);return BT::NodeStatus::FAILURE;}
   path.poses.resize(2);path.poses.front().pose.position.x=11.817;
   path.poses.front().pose.position.y=3.663;path.poses.back()=goal;
   n.setOutput("path",path);return BT::NodeStatus::SUCCESS;
 },{BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),BT::OutputPort<nav_msgs::msg::Path>("path"),
    BT::InputPort<std::string>("planner_id"),BT::OutputPort<int>("error_code_id")});
 std::ifstream stream(argv[2]);std::stringstream buffer;buffer<<stream.rdbuf();const auto xml=buffer.str();
 const auto begin=xml.find("<RecoveryNode number_of_retries=\"1\" name=\"FollowPath\">");
 if(begin==std::string::npos)throw std::runtime_error("Production FollowPath recovery missing");
 const auto end=xml.find("</RecoveryNode>",begin);
 const auto subtree=xml.substr(begin,end+std::string("</RecoveryNode>").size()-begin);
 auto run=[&](bool planner_fails,bool recoverable,int failures){
   state=Fixture{};state.planner_fails=planner_fails;state.recoverable=recoverable;state.fail_times=failures;
   auto bb=BT::Blackboard::create();nav_msgs::msg::Path old;old.poses.resize(2);
   bb->set("path",old);bb->set("follow_path_error_code",105);bb->set("compute_path_error_code",0);
   geometry_msgs::msg::PoseStamped goal;goal.pose.position.x=1.7;goal.pose.position.y=4.22;bb->set("goal",goal);
   const std::string text="<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\">"+subtree+"</BehaviorTree></root>";
   auto tree=factory.createTreeFromText(text,bb);
   const auto result=tree.tickWhileRunning();
   if(failures==0){
     if(result!=BT::NodeStatus::SUCCESS || state.plans || state.clears)throw std::runtime_error("Healthy path replanned");
   } else if(planner_fails || !recoverable){
     if(result!=BT::NodeStatus::FAILURE || state.follow!=1)throw std::runtime_error("Retried after failed recovery");
     nav_msgs::msg::Path output;bb->get("path",output);
     if(planner_fails && !output.poses.empty())throw std::runtime_error("Planner failure retained stale path");
     if(!recoverable && state.plans)throw std::runtime_error("Non-recoverable failure was replanned");
   } else if(result!=BT::NodeStatus::SUCCESS || state.plans!=1 || state.clears!=1 || state.follow!=2) {
     throw std::runtime_error("Tracking failure did not get one fresh plan before retry");
   }
 };
 run(false,true,0);run(false,true,1);run(true,true,1);run(false,false,1);
 std::cout<<"PASS: healthy route retained; failed tracking replans from live pose; failed planning stops; unrecoverable error stops\n";
}
