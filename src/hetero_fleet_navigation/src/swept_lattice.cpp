// State Lattice A* using Nav2-format control sets and swept rectangular bodies.
// Every edge is validated before relaxation, including pure rotations and reverse.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <cstdint>
#include <type_traits>
#include <queue>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_core/planner_exceptions.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_util/node_utils.hpp>
#include <nav2_util/line_iterator.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace hetero_fleet_navigation
{
namespace
{
constexpr double PI = 3.14159265358979323846;
struct Pose {double x, y, yaw;};
struct Cell {int x, y;};
struct Primitive
{
  int heading, dx, dy;
  double length, rotation;
  bool reverse;
  std::vector<Pose> poses;
  std::vector<Cell> swept;
};
double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}

// Enumerate *all* cells touching a rectangle, including occupied cell interiors.
// Positions use a cell-centred frame, so translating by an integer cell is exact.
std::vector<Cell> rectangleCells(Pose p, double hx, double hy, double resolution)
{
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  const double ax = std::abs(c) * hx + std::abs(s) * hy;
  const double ay = std::abs(s) * hx + std::abs(c) * hy;
  const double cell_axis = resolution / 2 * (std::abs(c) + std::abs(s));
  std::vector<Cell> cells;
  for (int j = std::floor((p.y - ay) / resolution - .5);
    j <= std::ceil((p.y + ay) / resolution + .5); ++j)
  {
    for (int i = std::floor((p.x - ax) / resolution - .5);
      i <= std::ceil((p.x + ax) / resolution + .5); ++i)
    {
      const double dx = i * resolution - p.x, dy = j * resolution - p.y;
      if (std::abs(dx) <= ax + resolution / 2 &&
        std::abs(dy) <= ay + resolution / 2 &&
        std::abs(c * dx + s * dy) <= hx + cell_axis &&
        std::abs(-s * dx + c * dy) <= hy + cell_axis)
      {
        cells.push_back({i, j});
      }
    }
  }
  return cells;
}
// Match the digital boundary used by planner_server's IsPathValid service.
// Bresenham can visit a neighbouring cell outside the exact continuous polygon.
std::vector<Cell> rasterCells(Pose p, double hx, double hy, double resolution)
{
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  const double xs[] = {hx, hx, -hx, -hx}, ys[] = {hy, -hy, -hy, hy};
  Cell corners[4];
  for (int i = 0; i < 4; ++i) {
    corners[i] = {static_cast<int>(std::floor(.5 + (p.x + c * xs[i] - s * ys[i]) / resolution)),
      static_cast<int>(std::floor(.5 + (p.y + s * xs[i] + c * ys[i]) / resolution))};
  }
  std::vector<Cell> cells;
  const int edges[][2] = {{0, 1}, {1, 2}, {2, 3}, {0, 3}};
  for (const auto & edge : edges) {
    const auto & a = corners[edge[0]]; const auto & b = corners[edge[1]];
    for (nav2_util::LineIterator line(a.x, a.y, b.x, b.y); line.isValid(); line.advance()) {
      cells.push_back({line.getX(), line.getY()});
    }
  }
  return cells;
}
std::vector<Pose> densify(const std::vector<Pose> & poses, double radius, double step)
{
  std::vector<Pose> result;
  if (poses.empty()) {return result;}
  result.push_back(poses.front());
  for (size_t i = 1; i < poses.size(); ++i) {
    const auto & a = poses[i - 1]; const auto & b = poses[i];
    const double dyaw = wrap(b.yaw - a.yaw);
    const int n = std::max(1, static_cast<int>(std::ceil(
          (std::hypot(b.x - a.x, b.y - a.y) + radius * std::abs(dyaw)) / step)));
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      result.push_back({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.yaw + t * dyaw});
    }
  }
  return result;
}
}

class SweptLattice : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    node_ = parent.lock(); costmap_ros_ = costmap; map_ = costmap->getCostmap();
    frame_ = costmap->getGlobalFrameID();
    auto param = [&](const std::string & key, const auto & value) {
        nav2_util::declare_parameter_if_not_declared(node_, name + "." + key,
          rclcpp::ParameterValue(value));
        return node_->get_parameter(name + "." + key).get_value<std::decay_t<decltype(value)>>();
      };
    const auto file = param("lattice_filepath", std::string());
    reverse_ = param("allow_reverse_expansion", false);
    unknown_ = param("allow_unknown", false);
    match_goal_heading_ = param("match_goal_heading", true);
    tolerance_ = param("tolerance", .05);
    max_time_ = param("max_planning_time", 8.0);
    max_iterations_ = param("max_iterations", 1000000);
    reverse_penalty_ = param("reverse_penalty", 2.0);
    cost_penalty_ = param("cost_penalty", 2.0);
    rotation_penalty_ = param("rotation_penalty", 2.0);
    heuristic_weight_ = param("heuristic_weight", 1.5);
    const auto footprint_text = param("swept_footprint", std::string());
    const auto padding = param("swept_footprint_padding", 0.0);
    if (tolerance_ < 0 || max_time_ <= 0 || max_iterations_ <= 0 ||
      reverse_penalty_ < 1 || cost_penalty_ < 0 || rotation_penalty_ <= 0 || heuristic_weight_ < 1 || padding < 0)
    {throw std::runtime_error("Invalid State Lattice search parameters");}
    nav2_costmap_2d::Footprint footprint;
    if (!nav2_costmap_2d::makeFootprintFromString(footprint_text, footprint) || footprint.size() != 4) {
      throw std::runtime_error("swept_footprint must be a centred rectangular body");
    }
    hx_ = hy_ = 0;
    for (const auto & p : footprint) {hx_ = std::max(hx_, std::abs(p.x)); hy_ = std::max(hy_, std::abs(p.y));}
    if (hx_ <= 0 || hy_ <= 0) {throw std::runtime_error("Empty swept footprint");}
    for (const auto & p : footprint) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
        std::abs(std::abs(p.x) - hx_) > 1e-6 || std::abs(std::abs(p.y) - hy_) > 1e-6) {
        throw std::runtime_error("swept_footprint must have rectangular corners about the origin");
      }
    }
    if (!std::isfinite(padding) || !std::isfinite(tolerance_) || !std::isfinite(max_time_) ||
      !std::isfinite(reverse_penalty_) || !std::isfinite(cost_penalty_) ||
      !std::isfinite(rotation_penalty_) || !std::isfinite(heuristic_weight_))
    {throw std::runtime_error("State Lattice parameters must be finite");}
    hx_ += padding; hy_ += padding;
    radius_ = std::hypot(hx_, hy_);
    std::ifstream input(file);
    if (!input) {throw std::runtime_error("Cannot open lattice control set: " + file);}
    nlohmann::json data; input >> data;
    auto metadata = data.at("lattice_metadata");
    resolution_ = metadata.at("grid_resolution");
    if (std::abs(resolution_ - map_->getResolution()) > 1e-9) {
      throw std::runtime_error("Lattice resolution must match the global costmap");
    }
    const std::string model = metadata.at("motion_model");
    if (model != "diff" && model != "omni") {throw std::runtime_error("Unsupported lattice motion model");}
    if (model == "omni" && reverse_) {
      throw std::runtime_error("Omni control sets contain explicit backwards primitives");
    }
    angles_ = metadata.at("heading_angles").get<std::vector<double>>();
    if (angles_.size() < 4 || angles_.size() != metadata.at("num_of_headings").get<size_t>()) {
      throw std::runtime_error("Invalid lattice headings");
    }
    for (size_t i = 0; i < angles_.size(); ++i) {
      if (!std::isfinite(angles_[i]) || angles_[i] < 0 || angles_[i] >= 2 * PI ||
        (i && angles_[i] <= angles_[i - 1])) {throw std::runtime_error("Invalid heading angles");}
    }
    if (data.at("primitives").size() != metadata.at("number_of_trajectories").get<size_t>()) {
      throw std::runtime_error("Incorrect lattice trajectory count");
    }
    primitives_.clear(); primitives_.resize(angles_.size());
    for (const auto & raw : data.at("primitives")) {
      const int start = raw.at("start_angle_index"), end = raw.at("end_angle_index");
      if (start < 0 || end < 0 || start >= static_cast<int>(angles_.size()) ||
        end >= static_cast<int>(angles_.size())) {throw std::runtime_error("Invalid primitive heading");}
      Primitive p; p.heading = end; p.reverse = false;
      for (const auto & pose : raw.at("poses")) {
        const Pose sample{pose.at(0), pose.at(1), pose.at(2)};
        if (!std::isfinite(sample.x) || !std::isfinite(sample.y) || !std::isfinite(sample.yaw)) {
          throw std::runtime_error("Primitive poses must be finite");
        }
        p.poses.push_back(sample);
      }
      if (p.poses.size() < 2 || std::hypot(p.poses.front().x, p.poses.front().y) > 1e-6 ||
        std::abs(wrap(p.poses.front().yaw - angles_[start])) > 1e-6 ||
        std::abs(wrap(p.poses.back().yaw - angles_[end])) > 1e-6)
      {throw std::runtime_error("Invalid primitive poses");}
      p.dx = std::lround(p.poses.back().x / resolution_);
      p.dy = std::lround(p.poses.back().y / resolution_);
      if (std::abs(p.dx * resolution_ - p.poses.back().x) > 1e-6 ||
        std::abs(p.dy * resolution_ - p.poses.back().y) > 1e-6)
      {throw std::runtime_error("Primitive endpoints must lie on the lattice");}
      p.reverse = model == "omni" && start == end &&
        p.dx * std::cos(angles_[start]) + p.dy * std::sin(angles_[start]) <
        -.5 * std::hypot(p.dx, p.dy);
      p.length = p.rotation = 0;
      for (size_t i = 1; i < p.poses.size(); ++i) {
        const auto & a = p.poses[i - 1]; const auto & b = p.poses[i];
        p.length += std::hypot(b.x - a.x, b.y - a.y);
        p.rotation += std::abs(wrap(b.yaw - a.yaw));
      }
      addPrimitive(start, p);
      if (reverse_ && p.length > 1e-6) {
        p.reverse = true; p.dx = -p.dx; p.dy = -p.dy;
        for (auto & pose : p.poses) {pose.x = -pose.x; pose.y = -pose.y;}
        addPrimitive(start, p);
      }
    }
    for (const auto & group : primitives_) {
      if (group.empty()) {throw std::runtime_error("Every heading needs motion primitives");}
    }
    RCLCPP_INFO(node_->get_logger(), "State Lattice configured: %zu headings, swept body %.3f x %.3f m",
      angles_.size(), hx_ * 2, hy_ * 2);
  }
  void cleanup() override {primitives_.clear(); angles_.clear(); costmap_ros_.reset(); node_.reset();}
  void activate() override {}
  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> cancel) override
  {
    const auto started = std::chrono::steady_clock::now();
    if (cancel()) {throw nav2_core::PlannerCancelled("State Lattice search cancelled");}
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map_->getMutex());
    if (std::abs(resolution_ - map_->getResolution()) > 1e-9) {
      throw nav2_core::PlannerException("Costmap resolution changed; reconfigure State Lattice");
    }
    const Pose start_pose{start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation)};
    Pose goal_pose{goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation)};
    if (!poseFree(start_pose)) {throw nav2_core::StartOccupied("Start body intersects an obstacle");}
    bool goal_free = match_goal_heading_ && poseFree(goal_pose);
    if (!match_goal_heading_) {
      for (const double yaw : angles_) {
        if (poseFree({goal_pose.x, goal_pose.y, yaw})) {goal_free = true; break;}
      }
    }
    if (!goal_free) {throw nav2_core::GoalOccupied("Goal body intersects an obstacle");}
    unsigned int sx, sy, gx, gy;
    if (!map_->worldToMap(start_pose.x, start_pose.y, sx, sy)) {throw nav2_core::StartOutsideMapBounds("Start outside map");}
    if (!map_->worldToMap(goal_pose.x, goal_pose.y, gx, gy)) {throw nav2_core::GoalOutsideMapBounds("Goal outside map");}
    width_ = map_->getSizeInCellsX(); height_ = map_->getSizeInCellsY();
    const unsigned int bins = angles_.size();
    const size_t count = static_cast<size_t>(width_) * height_ * bins;
    if (count >= std::numeric_limits<uint32_t>::max()) {throw nav2_core::PlannerException("Lattice too large");}
    const auto heuristic = obstacleHeuristic(gx, gy);
    const float inf = std::numeric_limits<float>::infinity();
    std::vector<float> costs(count, inf);
    std::vector<uint8_t> closed(count, 0);
    std::vector<uint32_t> parents(count, UINT32_MAX);
    std::vector<int> edges(count, -1);
    struct Entry {double f; float g; uint32_t id; bool operator<(const Entry & b) const {return f > b.f;}};
    std::priority_queue<Entry> open;
    // A sub-cell connector is permitted within the map resolution, as in Nav2.
    // Validate its full sweep, including quantizing the requested heading.
    const int start_heading = closest(start_pose.yaw), goal_heading = closest(goal_pose.yaw);
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        const int x = static_cast<int>(sx) + dx, y = static_cast<int>(sy) + dy;
        if (!inside(x, y)) {continue;}
        const Pose p = worldPose(x, y, start_heading);
        if (std::hypot(p.x - start_pose.x, p.y - start_pose.y) > resolution_ ||
          !sweepFree({start_pose, p})) {continue;}
        const auto id = index(x, y, start_heading);
        costs[id] = std::hypot(p.x - start_pose.x, p.y - start_pose.y);
        open.push({costs[id] + heuristic_weight_ * heuristic[y * width_ + x], costs[id], id});
      }
    }
    uint32_t finish = UINT32_MAX;
    size_t iterations = 0;
    while (!open.empty()) {
      const auto entry = open.top(); open.pop();
      if (entry.g != costs[entry.id] || closed[entry.id]) {continue;}
      closed[entry.id] = 1;
      const int heading = entry.id % bins;
      const int cell = entry.id / bins, x = cell % width_, y = cell / width_;
      if ((++iterations % 128) == 0) {
        if (cancel()) {throw nav2_core::PlannerCancelled("State Lattice search cancelled");}
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count() > max_time_) {
          throw nav2_core::PlannerTimedOut("State Lattice planning time exceeded after " + std::to_string(iterations) + " expansions");
        }
      }
      if (iterations > static_cast<size_t>(max_iterations_)) {break;}
      const Pose current = worldPose(x, y, heading);
      const Pose terminal{goal_pose.x, goal_pose.y, match_goal_heading_ ? goal_pose.yaw : current.yaw};
      if ((!match_goal_heading_ || heading == goal_heading) &&
        std::hypot(current.x - goal_pose.x, current.y - goal_pose.y) <= tolerance_ &&
        sweepFree({current, terminal})) {goal_pose = terminal; finish = entry.id; break;}
      for (size_t edge = 0; edge < primitives_[heading].size(); ++edge) {
        const auto & p = primitives_[heading][edge];
        const int nx = x + p.dx, ny = y + p.dy;
        if (!inside(nx, ny)) {continue;}
        const auto next = index(nx, ny, p.heading);
        if (closed[next]) {continue;}
        const double distance = p.length > 1e-6 ? p.length : radius_ * p.rotation * rotation_penalty_;
        const float lower = entry.g + distance * (p.reverse ? reverse_penalty_ : 1.0);
        if (lower >= costs[next]) {continue;}
        bool valid = true;
        for (const auto & c : p.swept) {
          if (!cellFree(x + c.x, y + c.y)) {valid = false; break;}
        }
        if (!valid) {continue;}
        unsigned char max_cost = 0;
        for (const auto & pose : p.poses) {
          const int cx = x + std::lround(pose.x / resolution_);
          const int cy = y + std::lround(pose.y / resolution_);
          if (inside(cx, cy)) {max_cost = std::max(max_cost, map_->getCost(cx, cy));}
        }
        const double norm = max_cost / 252.0;
        const float cost = entry.g + distance * (1 + cost_penalty_ * std::min(1.0, norm)) *
          (p.reverse ? reverse_penalty_ : 1.0);
        if (cost >= costs[next]) {continue;}
        costs[next] = cost; parents[next] = entry.id; edges[next] = edge;
        open.push({cost + heuristic_weight_ * heuristic[ny * width_ + nx], cost, next});
      }
    }
    if (finish == UINT32_MAX) {throw nav2_core::NoValidPathCouldBeFound("No swept-body State Lattice route");}
    std::vector<uint32_t> chain;
    for (uint32_t i = finish; i != UINT32_MAX; i = parents[i]) {chain.push_back(i);}
    std::reverse(chain.begin(), chain.end());
    nav_msgs::msg::Path path; path.header.frame_id = frame_; path.header.stamp = node_->now();
    auto append = [&](Pose p) {
        geometry_msgs::msg::PoseStamped pose; pose.header = path.header;
        pose.pose.position.x = p.x; pose.pose.position.y = p.y;
        pose.pose.orientation.z = std::sin(p.yaw / 2); pose.pose.orientation.w = std::cos(p.yaw / 2);
        path.poses.push_back(pose);
      };
    append(start_pose);
    const auto first = chain.front();
    append(worldPose((first / bins) % width_, (first / bins) / width_, first % bins));
    for (size_t i = 1; i < chain.size(); ++i) {
      const auto previous = chain[i - 1]; const auto id = chain[i];
      const auto base = worldPose((previous / bins) % width_, (previous / bins) / width_, previous % bins);
      const auto & primitive = primitives_[previous % bins][edges[id]];
      for (size_t j = 1; j < primitive.poses.size(); ++j) {
        const auto & p = primitive.poses[j]; append({base.x + p.x, base.y + p.y, p.yaw});
      }
    }
    append(goal_pose);
    RCLCPP_DEBUG(node_->get_logger(), "State Lattice: %zu expansions, %zu poses", iterations, path.poses.size());
    return path;
  }

private:
  void addPrimitive(int start, Primitive p)
  {
    // Half-step padding covers the continuous motion between sweep samples.
    // The corner travel is bounded by translation + radius * angular change.
    const double step = resolution_ / 8, extra = step / 2;
    const auto samples = densify(p.poses, radius_, step);
    p.poses = densify(p.poses, radius_, resolution_);
    p.poses.front().yaw = angles_[start]; p.poses.back().yaw = angles_[p.heading];
    double min_x = 0, max_x = 0, min_y = 0, max_y = 0;
    for (const auto & p : samples) {
      min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
    }
    const int x0 = std::floor((min_x - radius_ - extra) / resolution_) - 1;
    const int y0 = std::floor((min_y - radius_ - extra) / resolution_) - 1;
    const int w = std::ceil((max_x + radius_ + extra) / resolution_) - x0 + 2;
    const int h = std::ceil((max_y + radius_ + extra) / resolution_) - y0 + 2;
    std::vector<uint8_t> mask(w * h, 0);
    for (const auto & pose : samples) {
      for (const auto & c : rectangleCells(pose, hx_ + extra, hy_ + extra, resolution_)) {
        mask[(c.y - y0) * w + c.x - x0] = 1;
      }
    }
    for (const auto & poses : {samples, p.poses}) {
      for (const auto & pose : poses) {
        for (const auto & c : rasterCells(pose, hx_, hy_, resolution_)) {
          mask[(c.y - y0) * w + c.x - x0] = 1;
        }
      }
    }
    // The source body's cells were validated by the incoming edge (or seed
    // connector). Only newly swept cells need checking for an outgoing edge.
    for (const auto & c : rectangleCells({0, 0, angles_[start]}, hx_ + extra, hy_ + extra, resolution_)) {
      mask[(c.y - y0) * w + c.x - x0] = 0;
    }
    for (const auto & c : rasterCells({0, 0, angles_[start]}, hx_, hy_, resolution_)) {
      mask[(c.y - y0) * w + c.x - x0] = 0;
    }
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        if (mask[y * w + x]) {p.swept.push_back({x + x0, y + y0});}
      }
    }
    // Check the centre first, then cells near the body's outline, to reject fast.
    std::sort(p.swept.begin(), p.swept.end(), [](const Cell & a, const Cell & b) {
        return a.x * a.x + a.y * a.y < b.x * b.x + b.y * b.y;
      });
    primitives_[start].push_back(std::move(p));
  }
  bool inside(int x, int y) const
  {return x >= 0 && y >= 0 && x < static_cast<int>(map_->getSizeInCellsX()) && y < static_cast<int>(map_->getSizeInCellsY());}
  bool cellFree(int x, int y) const
  {
    if (!inside(x, y)) {return false;}
    const auto cost = map_->getCost(x, y);
    // Inscribed inflation is a centre heuristic, not a physical obstacle.
    return cost == nav2_costmap_2d::NO_INFORMATION ? unknown_ : cost < nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  bool poseFree(Pose p, double extra = 0) const
  {
    p.x -= map_->getOriginX() + resolution_ / 2;
    p.y -= map_->getOriginY() + resolution_ / 2;
    for (const auto & c : rectangleCells(p, hx_ + extra, hy_ + extra, resolution_)) {
      if (!cellFree(c.x, c.y)) {return false;}
    }
    return true;
  }
  bool sweepFree(const std::vector<Pose> & poses) const
  {
    const double step = resolution_ / 8;
    for (const auto & p : densify(poses, radius_, step)) {
      if (!poseFree(p, step / 2)) {return false;}
      Pose relative = p;
      relative.x -= map_->getOriginX() + resolution_ / 2;
      relative.y -= map_->getOriginY() + resolution_ / 2;
      for (const auto & cell : rasterCells(relative, hx_, hy_, resolution_)) {
        if (!cellFree(cell.x, cell.y)) {return false;}
      }
    }
    return true;
  }
  int closest(double yaw) const
  {
    int best = 0;
    for (size_t i = 1; i < angles_.size(); ++i) {
      if (std::abs(wrap(angles_[i] - yaw)) < std::abs(wrap(angles_[best] - yaw))) {best = i;}
    }
    return best;
  }
  uint32_t index(int x, int y, int heading) const
  {return (y * width_ + x) * angles_.size() + heading;}
  Pose worldPose(int x, int y, int heading) const
  {return {map_->getOriginX() + (x + .5) * resolution_, map_->getOriginY() + (y + .5) * resolution_, angles_[heading]};}
  std::vector<float> obstacleHeuristic(int gx, int gy) const
  {
    const float inf = std::numeric_limits<float>::infinity();
    std::vector<float> values(width_ * height_, inf);
    using Item = std::pair<float, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    const int goal = gy * width_ + gx; values[goal] = 0; open.emplace(0, goal);
    while (!open.empty()) {
      const auto [cost, cell] = open.top(); open.pop();
      if (cost != values[cell]) {continue;}
      const int x = cell % width_, y = cell / width_;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int nx = x + dx, ny = y + dy;
          if ((!dx && !dy) || !cellFree(nx, ny)) {continue;}
          // The inflation layer encodes the configured planning envelope's
          // inscribed disk. This avoids heuristic shortcuts through tight walls.
          const auto centre_cost = map_->getCost(nx, ny);
          if (centre_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
            !(unknown_ && centre_cost == nav2_costmap_2d::NO_INFORMATION)) {continue;}
          const int next = ny * width_ + nx;
          const double norm = std::min(map_->getCost(nx, ny), map_->getCost(x, y)) / 252.0;
          const float distance = cost + resolution_ * std::hypot(dx, dy) *
            (1 + cost_penalty_ * std::min(1.0, norm));
          if (distance < values[next]) {values[next] = distance; open.emplace(distance, next);}
        }
      }
    }
    // Infinity correctly excludes disconnected regions but never creates an unsafe edge.
    return values;
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * map_ = nullptr;
  std::string frame_;
  std::vector<double> angles_;
  std::vector<std::vector<Primitive>> primitives_;
  int width_ = 0, height_ = 0, max_iterations_ = 1000000;
  double hx_, hy_, radius_, resolution_, tolerance_, max_time_;
  double reverse_penalty_, cost_penalty_, rotation_penalty_, heuristic_weight_;
  bool reverse_, unknown_, match_goal_heading_;
};
}
PLUGINLIB_EXPORT_CLASS(hetero_fleet_navigation::SweptLattice, nav2_core::GlobalPlanner)
