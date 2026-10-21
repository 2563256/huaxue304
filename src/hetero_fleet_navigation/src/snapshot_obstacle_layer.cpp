#include <algorithm>
#include <mutex>
#include <nav2_costmap_2d/obstacle_layer.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace hetero_fleet_navigation
{
// Fleet scans describe the CURRENT complete peer configuration. Ray tracing
// alone cannot erase old marks reliably: angular gaps at long range leave
// isolated lethal cells after a peer moves, even with observation_persistence=0.
// Rebuild only this layer from its latest buffered observation every update.
// Static-map and physical-lidar layers retain their normal persistence.
class SnapshotObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void updateBounds(double x, double y, double yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());
    resetMaps();
    nav2_costmap_2d::ObstacleLayer::updateBounds(x, y, yaw, min_x, min_y, max_x, max_y);
    // Invalidate the old master cells as well, including old inflation. Merely
    // clearing our private grid would leave marks in the merged master map.
    *min_x = std::min(*min_x, getOriginX());
    *min_y = std::min(*min_y, getOriginY());
    *max_x = std::max(*max_x, getOriginX() + getSizeInCellsX() * getResolution());
    *max_y = std::max(*max_y, getOriginY() + getSizeInCellsY() * getResolution());
  }
};
}
PLUGINLIB_EXPORT_CLASS(hetero_fleet_navigation::SnapshotObstacleLayer, nav2_costmap_2d::Layer)
