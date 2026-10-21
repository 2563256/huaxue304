// Run the installed Nav2 planners against the real static map without Gazebo.
#include <algorithm>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <nav2_map_server/map_io.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <pluginlib/class_loader.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

using Json = nlohmann::json;

geometry_msgs::msg::PoseStamped pose(const YAML::Node & input)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = input[0].as<double>();
  result.pose.position.y = input[1].as<double>();
  const double yaw = input[2].as<double>();
  result.pose.orientation.z = std::sin(yaw / 2.0);
  result.pose.orientation.w = std::cos(yaw / 2.0);
  return result;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto spec = YAML::LoadFile(argv[1]);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("scene_planner_probe");
  const std::vector<std::string> bools = {
    "allow_unknown", "allow_reverse_expansion", "smooth_path", "downsample_costmap",
    "allow_primitive_interpolation", "cache_obstacle_heuristic", "match_goal_heading",
    "use_final_approach_orientation"};
  const std::vector<std::string> ints = {
    "angle_quantization_bins", "max_iterations", "max_on_approach_iterations"};
  const std::vector<std::string> strings = {"motion_model_for_search", "lattice_filepath", "swept_footprint"};
  for (const auto & entry : spec["planner"]) {
    const auto key = entry.first.as<std::string>();
    if (key == "plugin") {continue;}
    const auto name = "GridBased." + key;
    if (std::find(bools.begin(), bools.end(), key) != bools.end()) {
      node->declare_parameter(name, entry.second.as<bool>());
    } else if (std::find(ints.begin(), ints.end(), key) != ints.end()) {
      node->declare_parameter(name, entry.second.as<int>());
    } else if (std::find(strings.begin(), strings.end(), key) != strings.end()) {
      node->declare_parameter(name, entry.second.as<std::string>());
    } else {
      node->declare_parameter(name, entry.second.as<double>());
    }
  }

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("probe_costmap");
  costmap->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{"inflation_layer"}));
  costmap->set_parameter(rclcpp::Parameter("global_frame", "map"));
  costmap->set_parameter(rclcpp::Parameter("footprint_padding", spec["padding"].as<double>()));
  costmap->declare_parameter("inflation_layer.plugin", std::string("nav2_costmap_2d::InflationLayer"));
  costmap->declare_parameter("inflation_layer.inflation_radius", spec["inflation_radius"].as<double>());
  costmap->declare_parameter("inflation_layer.cost_scaling_factor", spec["scaling"].as<double>());
  costmap->set_parameter(rclcpp::Parameter("footprint", spec["footprint"].as<std::string>()));
  costmap->on_configure(rclcpp_lifecycle::State());
  if (costmap->getUseRadius()) {throw std::runtime_error("Expected polygon collision checking");}

  nav_msgs::msg::OccupancyGrid grid;
  if (nav2_map_server::loadMapFromYaml(spec["map"].as<std::string>(), grid) !=
    nav2_map_server::LOAD_MAP_STATUS::LOAD_MAP_SUCCESS)
  {
    return 2;
  }
  auto * map = costmap->getCostmap();
  costmap->getLayeredCostmap()->resizeMap(
    grid.info.width, grid.info.height, grid.info.resolution,
    grid.info.origin.position.x, grid.info.origin.position.y);
  for (size_t index = 0; index < grid.data.size(); ++index) {
    map->getCharMap()[index] = grid.data[index] < 0 ? 255 : (grid.data[index] >= 65 ? 254 : 0);
  }
  double min_x = grid.info.origin.position.x, min_y = grid.info.origin.position.y;
  double max_x = min_x + grid.info.width * grid.info.resolution;
  double max_y = min_y + grid.info.height * grid.info.resolution;
  for (auto & layer : *costmap->getLayeredCostmap()->getPlugins()) {
    layer->updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
    layer->updateCosts(*map, 0, 0, grid.info.width, grid.info.height);
  }

  // Load only the selected plugin, as a production planner_server does.
  // Linking both Smac libraries directly aliases their static heuristic caches.
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
  auto planner = loader.createUniqueInstance(spec["planner"]["plugin"].as<std::string>());
  planner->configure(node, "GridBased", nullptr, costmap);
  planner->activate();
  Json results = Json::array();
  const std::vector<unsigned char> base_costs(
    map->getCharMap(), map->getCharMap() + map->getSizeInCellsX() * map->getSizeInCellsY());
  const bool reinflate = spec["reinflate_obstacles"] && spec["reinflate_obstacles"].as<bool>();
  for (const auto & request : spec["cases"]) {
    if (reinflate) {std::copy(base_costs.begin(), base_costs.end(), map->getCharMap());}
    std::cerr << "Planning " << request["name"].as<std::string>() << std::endl;
    Json item;
    item["name"] = request["name"].as<std::string>();
    std::vector<std::pair<unsigned int, unsigned char>> saved;
    for (const auto & obstacle : request["obstacles"]) {
      unsigned int x, y;
      if (!map->worldToMap(obstacle[0].as<double>(), obstacle[1].as<double>(), x, y)) {
        throw std::runtime_error("Probe obstacle outside map");
      }
      const auto offset = map->getIndex(x, y);
      saved.emplace_back(offset, map->getCharMap()[offset]);
      map->getCharMap()[offset] = nav2_costmap_2d::LETHAL_OBSTACLE;
    }
    if (reinflate) {
      // Match production: dynamic obstacle layers run before inflation.
      for (auto & layer : *costmap->getLayeredCostmap()->getPlugins()) {
        layer->updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
        layer->updateCosts(*map, 0, 0, grid.info.width, grid.info.height);
      }
    }
    const bool cancelled = request["cancel"] && request["cancel"].as<bool>();
    const auto begun = std::chrono::steady_clock::now();
    try {
      const auto path = planner->createPlan(pose(request["start"]), pose(request["goal"]), [cancelled] {return cancelled;});
      item["ok"] = !path.poses.empty();
      item["poses"] = Json::array();
      item["nav2_footprint_valid"] = true;
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
      for (const auto & point : path.poses) {
        const auto & q = point.pose.orientation;
        const auto yaw = 2 * std::atan2(q.z, q.w);
        item["poses"].push_back({point.pose.position.x, point.pose.position.y, yaw});
        const auto cost = checker.footprintCostAtPose(
          point.pose.position.x, point.pose.position.y, yaw, costmap->getRobotFootprint());
        if (cost < 0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
          item["nav2_footprint_valid"] = false;
        }
      }
    } catch (const std::exception & exception) {
      item["ok"] = false;
      item["error"] = exception.what();
    }
    item["planning_seconds"] = std::chrono::duration<double>(std::chrono::steady_clock::now() - begun).count();
    std::cerr << "Result " << item["name"] << ": " << item["ok"] << " (" << item["planning_seconds"] << "s)";
    if (item.contains("error")) {std::cerr << " " << item["error"];}
    std::cerr << std::endl;
    results.push_back(item);
    for (const auto & cell : saved) {map->getCharMap()[cell.first] = cell.second;}
  }
  std::ofstream output(spec["output"].as<std::string>());
  output << results.dump();
  planner->deactivate();
  planner->cleanup();
  planner.reset();
  costmap->on_cleanup(rclcpp_lifecycle::State());
  costmap.reset();
  node.reset();
  rclcpp::shutdown();
}
