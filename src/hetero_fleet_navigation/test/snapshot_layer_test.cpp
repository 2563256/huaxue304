#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.h>
#include <iostream>
#include <stdexcept>

class WallLayer : public nav2_costmap_2d::Layer
{
public:
 void onInitialize() override {current_ = enabled_ = true;}
 void reset() override {}
 bool isClearable() override {return false;}
 void updateBounds(double,double,double,double*,double*,double*,double*) override {}
 void updateCosts(nav2_costmap_2d::Costmap2D &map,int,int,int,int) override {
  unsigned int x,y; map.worldToMap(2.025,2.025,x,y); map.setCost(x,y,254);
 }
};

int main(int argc,char **argv)
{
 rclcpp::init(argc,argv);
 auto node=std::make_shared<nav2_util::LifecycleNode>("snapshot_layer_regression");
 node->declare_parameter("track_unknown_space",false);
 node->declare_parameter("transform_tolerance",.5);
 auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
 auto group=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
 nav2_costmap_2d::LayeredCostmap map("map",false,false);
 map.resizeMap(240,240,.05,0.,0.);
 auto wall=std::make_shared<WallLayer>(); map.addPlugin(wall);
 wall->initialize(&map,"wall",tf.get(),node,group);
 pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader("nav2_costmap_2d","nav2_costmap_2d::Layer");
 auto layer=loader.createSharedInstance("hetero_fleet_navigation::SnapshotObstacleLayer");
 map.addPlugin(layer); layer->initialize(&map,"peers",tf.get(),node,group);
 auto obstacle=std::dynamic_pointer_cast<nav2_costmap_2d::ObstacleLayer>(layer);
 node->declare_parameter("inflation.inflation_radius",.6);
 auto inflation=loader.createSharedInstance("nav2_costmap_2d::InflationLayer");
 map.addPlugin(inflation);inflation->initialize(&map,"inflation",tf.get(),node,group);
 auto cost=[&](double x,double y){unsigned int i,j;map.getCostmap()->worldToMap(x,y,i,j);return map.getCostmap()->getCost(i,j);};
 auto require=[](bool ok,const char *message){if(!ok)throw std::runtime_error(message);};
 auto observe=[&](double x,double y){
  obstacle->clearStaticObservations(true,true);
  sensor_msgs::msg::PointCloud2 cloud; cloud.header.frame_id="map";
  sensor_msgs::PointCloud2Modifier modifier(cloud); modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> px(cloud,"x"),py(cloud,"y"),pz(cloud,"z");
  *px=x;*py=y;*pz=.1;
  geometry_msgs::msg::Point origin; origin.x=1.;origin.y=1.;
  nav2_costmap_2d::Observation observation(origin,cloud,20.,0.,20.,0.);
  // No clearing rays: sparse scans need not traverse each old obstacle cell.
  obstacle->addStaticObservation(observation,true,false);
 };
 observe(8.025,5.025);map.updateMap(1.,1.,0.);
 require(cost(8.025,5.025)==254,"Current peer was not marked");
 require(cost(8.325,5.025)>0,"Current obstacle was not inflated");
 map.updateMap(1.,1.,0.);
 require(cost(8.025,5.025)==254,"Missing new scan discarded the last known obstacle");
 observe(8.025,7.025);map.updateMap(1.,1.,0.);
 require(cost(8.025,5.025)==0,"Old peer cell survived a changed snapshot");
 require(cost(8.325,5.025)==0,"Old inflation survived a changed snapshot");
 require(cost(8.025,7.025)==254,"Replacement peer was erased");
 require(cost(2.025,2.025)==254,"Rebuilding peers erased another layer's wall");
 obstacle->clearStaticObservations(true,true);map.updateMap(1.,1.,0.);
 require(cost(8.025,7.025)==0,"Empty current snapshot retained a ghost");
 require(cost(2.025,2.025)==254,"Empty snapshot cleared a static obstacle");
 std::cout<<"PASS: moving and stationary peers, empty snapshot, preserved independent wall\n";
 rclcpp::shutdown();
}
