// Preserve model identities discarded by the generic Pose_V -> TF converter.
#include <rclcpp/rclcpp.hpp>
#include <gz/transport/Node.hh>
#include <hetero_fleet_gz/named_poses.hpp>
#include <cstdint>
#include <mutex>

class NamedPoseBridge : public rclcpp::Node {
public:
  NamedPoseBridge() : Node("named_pose_bridge") {
    const auto names = declare_parameter<std::vector<std::string>>("robot_names", std::vector<std::string>{});
    models_.insert(names.begin(), names.end());
    const double rate = declare_parameter<double>("publish_rate_hz", 40.0);
    if (models_.empty() || rate <= 0.) {throw std::runtime_error("Expected robot names and positive pose rate");}
    period_ = static_cast<int64_t>(1e9/rate);
    output_ = create_publisher<tf2_msgs::msg::TFMessage>("/gz/named_dynamic_pose", 5);
    topic_ = declare_parameter<std::string>("gz_topic", "/world/geomap_fleet/dynamic_pose/info");
    if (!transport_.Subscribe(topic_, &NamedPoseBridge::receive, this)) {
      throw std::runtime_error("Cannot subscribe to Gazebo model poses");
    }
  }
  ~NamedPoseBridge() override {
    transport_.Unsubscribe(topic_);
    std::lock_guard<std::mutex> lock(mutex_);
  }
private:
  void receive(const gz::msgs::Pose_V & message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!rclcpp::ok()) {return;}
    const int64_t stamp = message.header().stamp().sec()*1000000000LL + message.header().stamp().nsec();
    if (last_ >= 0 && stamp >= last_ && stamp-last_ < period_) {return;}
    last_ = stamp;
    output_->publish(namedModelPoses(message, models_));
  }
  gz::transport::Node transport_;
  std::string topic_;
  std::mutex mutex_;
  std::unordered_set<std::string> models_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr output_;
  int64_t last_{-1}, period_{25000000};
};
int main(int argc, char ** argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<NamedPoseBridge>());
  rclcpp::shutdown(); return 0;
}
