#include <nav2_mppi_controller/critic_function.hpp>
#include <nav2_mppi_controller/tools/parameters_handler.hpp>
#include <pluginlib/class_loader.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main(int argc,char **argv)
{
 if(argc<2)return 2;
 rclcpp::init(argc,argv);
 auto config=YAML::LoadFile(argv[1])["controller_server"]["ros__parameters"]["FollowPath"];
 auto settings=config["PathAlignCritic"];
 auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("reverse_alignment_regression");
 auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>("alignment_costmap");
 map->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));
 map->on_configure(rclcpp_lifecycle::State());
 pluginlib::ClassLoader<mppi::critics::CriticFunction> loader("nav2_mppi_controller","mppi::critics::CriticFunction");
 mppi::models::State state;state.reset(2,56);state.pose.pose.position.x=5.;state.pose.pose.position.y=5.;
 state.pose.pose.orientation.z=1.;state.pose.pose.orientation.w=0.;
 mppi::models::Path path;path.reset(100);
 for(int i=0;i<100;++i){path.x(i)=5.+i*.05;path.y(i)=5.;path.yaws(i)=M_PI;}
 mppi::models::Trajectories trajectories;trajectories.reset(2,56);
 for(int row=0;row<2;++row)for(int t=0;t<56;++t){
  trajectories.x(row,t)=5.+.25*.05*(t+1);trajectories.y(row,t)=5.;
  trajectories.yaws(row,t)=M_PI-(row?.36:0.);
 }
 geometry_msgs::msg::Pose goal;goal.position.x=10.;goal.position.y=5.;
 float dt=.05;
 auto score=[&](const std::string &name,int offset,bool at_goal){
  const auto prefix=name+".PathAlignCritic";
  node->declare_parameter(prefix+".offset_from_furthest",offset);
  node->declare_parameter(prefix+".use_path_orientations",true);
  node->declare_parameter(prefix+".cost_weight",settings["cost_weight"].as<double>());
  node->declare_parameter(prefix+".threshold_to_consider",settings["threshold_to_consider"].as<double>());
  mppi::ParametersHandler handler(node);
  auto critic=loader.createSharedInstance("mppi::critics::PathAlignCritic");
  critic->on_configure(node,name,prefix,map,&handler);
  xt::xtensor<float,1> costs=xt::zeros<float>({2});
  auto local_goal=goal;if(at_goal){local_goal.position.x=5.1;local_goal.position.y=5.;}
  mppi::CriticData data{state,trajectories,path,local_goal,costs,dt,false,nullptr,nullptr,
                       std::vector<bool>(100,true),size_t(14)};
  critic->score(data);return costs;
 };
 auto require=[](bool ok,const char *message){if(!ok)throw std::runtime_error(message);};
 auto old=score("legacy",20,false);
 require(old(0)==0.&&old(1)==0.,"Expected legacy reverse horizon to disable alignment");
 auto fixed=score("fixed",settings["offset_from_furthest"].as<int>(),false);
 require(fixed(1)>fixed(0)+1.,"Reverse yaw drift is still unpenalized");
 auto arrived=score("arrived",settings["offset_from_furthest"].as<int>(),true);
 require(arrived(0)==0.&&arrived(1)==0.,"Arrival must not enforce a task heading");
 std::cout<<"PASS: reverse heading drift scores "<<old(1)<<" -> "<<fixed(1)
          <<"; within 0.15 m arrival alignment is disabled\n";
 rclcpp::shutdown();
}
