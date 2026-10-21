#pragma once
#include <string>
#include <unordered_set>
#include <gz/msgs/pose_v.pb.h>
#include <tf2_msgs/msg/tf_message.hpp>

inline tf2_msgs::msg::TFMessage namedModelPoses(
  const gz::msgs::Pose_V & source, const std::unordered_set<std::string> & models)
{
  tf2_msgs::msg::TFMessage result;
  for (const auto & pose : source.pose()) {
    if (!models.count(pose.name())) {continue;}
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map"; t.child_frame_id = pose.name();
    t.header.stamp.sec = source.header().stamp().sec();
    t.header.stamp.nanosec = source.header().stamp().nsec();
    t.transform.translation.x = pose.position().x();
    t.transform.translation.y = pose.position().y();
    t.transform.translation.z = pose.position().z();
    t.transform.rotation.x = pose.orientation().x();
    t.transform.rotation.y = pose.orientation().y();
    t.transform.rotation.z = pose.orientation().z();
    t.transform.rotation.w = pose.orientation().w();
    result.transforms.push_back(t);
  }
  return result;
}
