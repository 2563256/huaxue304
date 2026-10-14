#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <chrono>
#include <iostream>
#include <thread>
#include <stdexcept>
int main(int argc,char**argv) {
 if(argc<2)return 2;rclcpp::init(argc,argv);
 auto node=std::make_shared<rclcpp::Node>("stable_route_regression");
 auto clock=node->get_clock()->get_clock_handle();
 if(rcl_enable_ros_time_override(clock)!=RCL_RET_OK)throw std::runtime_error("Clock override failed");
 auto set_clock=[&](int64_t t){if(rcl_set_ros_time_override(clock,t)!=RCL_RET_OK)throw std::runtime_error("Clock set failed");};
 int64_t sim=1000000000LL;set_clock(sim);
 bool valid=false,changed=true;int plans=0;
 BT::BehaviorTreeFactory f;f.registerFromPlugin(argv[1]);
 f.registerSimpleCondition("GlobalUpdatedGoal",[&](BT::TreeNode&){bool old=changed;changed=false;return old?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;});
 f.registerSimpleCondition("IsPathValid",[&](BT::TreeNode&){return valid?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;});
 f.registerSimpleAction("ComputePathToPose",[&](BT::TreeNode& n){++plans;valid=true;nav_msgs::msg::Path p;p.header.stamp=node->now();p.poses.resize(2);n.setOutput("path",p);return BT::NodeStatus::SUCCESS;},{BT::OutputPort<nav_msgs::msg::Path>("path")});
 auto bb=BT::Blackboard::create();bb->set("node",node);bb->set("path",nav_msgs::msg::Path{});
 auto tree=f.createTreeFromText(R"(<root BTCPP_format="4"><BehaviorTree ID="Test"><Fallback><Sequence><GlobalUpdatedGoal/><ComputePathToPose path="{path}"/></Sequence><StableReplan path="{path}" blocked_duration="5.0" min_interval="15.0"><IsPathValid/><ComputePathToPose path="{path}"/></StableReplan></Fallback></BehaviorTree></root>)",bb);
 auto run=[&](double sec){int ticks=static_cast<int>(sec/.05);for(int i=0;i<ticks;++i){sim+=50000000LL;set_clock(sim);tree.tickOnce();}};
 tree.tickOnce();if(plans!=1)throw std::runtime_error("Initial plan not immediate");
 run(200.);if(plans!=1)throw std::runtime_error("Valid path periodically replanned");
 valid=false;run(2.);valid=true;run(5.);if(plans!=1)throw std::runtime_error("Transient traffic replanned");
 valid=false;run(4.9);if(plans!=1)throw std::runtime_error("Blockage debounce failed");
 run(.3);if(plans!=2)throw std::runtime_error("Persistent blockage not replanned");
 valid=false;run(8.);if(plans!=2)throw std::runtime_error("Cooldown failed under continuous invalidity");
 for(int i=0;i<20;++i){tree.tickOnce();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
 if(plans!=2)throw std::runtime_error("Paused simulation consumed ROS-time cooldown");
 run(8.);if(plans!=3)throw std::runtime_error("Cooldown prevented bounded persistent replan");
 changed=true;tree.tickOnce();if(plans!=4)throw std::runtime_error("Goal change waited for cooldown");
 valid=false;sim=0;set_clock(sim);run(5.5);
 if(plans!=5)throw std::runtime_error("Simulation reset stranded cooldown in the future");
 std::cout<<"PASS: initial plan, no expiry, transient traffic, persistent blockage, cooldown, immediate changed goal\n";rclcpp::shutdown();
}
