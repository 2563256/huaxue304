#include <hetero_fleet_gz/named_poses.hpp>
#include <stdexcept>
#include <iostream>
int main() {
  gz::msgs::Pose_V poses;
  auto add=[&](const std::string & name,double x) {
    auto p=poses.add_pose();p->set_name(name);p->mutable_position()->set_x(x);p->mutable_orientation()->set_w(1.);
  };
  add("heavy_1",2.7);add("base_link",0.);
  auto first=namedModelPoses(poses,{"omni_1","heavy_1"});
  if(first.transforms.size()!=1 || first.transforms[0].child_frame_id!="heavy_1")
    throw std::runtime_error("Missing model substituted by neighbour or nested link");
  add("omni_1",4.6);
  auto complete=namedModelPoses(poses,{"omni_1","heavy_1"});
  if(complete.transforms.size()!=2 || complete.transforms[1].child_frame_id!="omni_1" ||
      complete.transforms[1].transform.translation.x!=4.6)
    throw std::runtime_error("Wrong model identity/pose");
  poses.mutable_pose()->SwapElements(0,2);
  auto reordered=namedModelPoses(poses,{"omni_1","heavy_1"});
  if(reordered.transforms[0].child_frame_id!="omni_1" || reordered.transforms[0].transform.translation.x!=4.6)
    throw std::runtime_error("Pose ordering changed identity");
  std::cout<<"PASS: exact model names, missing model omitted, nested links rejected, reorder safe\n";
}
