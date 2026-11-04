#include "warotrans_lane_planner/lane_aware_navfn.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <unordered_map>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <nlohmann/json.hpp>

namespace warotrans_lane_planner
{
namespace
{

double normalizeAngle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

double distPointSeg(double px, double py, double ax, double ay, double bx, double by, double & t)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  if (len2 < 1e-12) {
    t = 0.0;
    return std::hypot(px - ax, py - ay);
  }
  t = ((px - ax) * dx + (py - ay) * dy) / len2;
  t = std::clamp(t, 0.0, 1.0);
  const double qx = ax + t * dx;
  const double qy = ay + t * dy;
  return std::hypot(px - qx, py - qy);
}

}  // namespace

void LaneAwareNavfn::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("LaneAwareNavfn: failed to lock node");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lanes_json", rclcpp::ParameterValue(std::string("")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".corridor_margin", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lethal_cost",
    rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".start_goal_lethal_cost",
    rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lane_prefer_factor", rclcpp::ParameterValue(0.15));

  std::string lanes_path;
  node->get_parameter(name_ + ".lanes_json", lanes_path);
  node->get_parameter(name_ + ".corridor_margin", corridor_margin_);
  node->get_parameter(name_ + ".lethal_cost", lethal_cost_);
  node->get_parameter(name_ + ".start_goal_lethal_cost", start_goal_lethal_cost_);
  node->get_parameter(name_ + ".lane_prefer_factor", lane_prefer_factor_);

  if (!lanes_path.empty()) {
    bool loaded = false;
    try {
      loaded = loadLanes(lanes_path);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node->get_logger(), "LaneAwareNavfn: %s is not valid lanes JSON: %s", lanes_path.c_str(), e.what());
    }
    if (!loaded) {
      // Fail closed: a configured lane file that cannot be read must not silently drop ONE_WAY
      // (the planner would then route against traffic). planner_server fails to configure instead.
      throw std::runtime_error("LaneAwareNavfn: failed to load lanes from " + lanes_path);
    } else {
      RCLCPP_INFO(
        node->get_logger(), "LaneAwareNavfn: loaded %zu lane segments from %s", segs_.size(),
        lanes_path.c_str());
    }
  } else {
    RCLCPP_WARN(node->get_logger(), "LaneAwareNavfn: lanes_json empty — ONE_WAY constraints disabled");
  }
}

void LaneAwareNavfn::cleanup() {}
void LaneAwareNavfn::activate() {}
void LaneAwareNavfn::deactivate() {}

bool LaneAwareNavfn::loadLanes(const std::string & path)
{
  segs_.clear();
  std::ifstream in(path);
  if (!in) {
    return false;
  }
  nlohmann::json root;
  in >> root;
  if (!root.contains("lanes") || !root["lanes"].is_array()) {
    return false;
  }
  for (const auto & lane : root["lanes"]) {
    const bool one_way = lane.value("direction", "BIDIRECTIONAL") == "ONE_WAY";
    const double width = lane.value("width_meters", 0.9);
    const double half = 0.5 * width + corridor_margin_;
    if (!lane.contains("tangent_samples") || !lane["tangent_samples"].is_array()) {
      continue;
    }
    for (const auto & s : lane["tangent_samples"]) {
      LaneSeg seg;
      seg.ax = s["a"][0].get<double>();
      seg.ay = s["a"][1].get<double>();
      seg.bx = s["b"][0].get<double>();
      seg.by = s["b"][1].get<double>();
      seg.tangent = s.value("tangent", std::atan2(seg.by - seg.ay, seg.bx - seg.ax));
      seg.half_width = half;
      seg.one_way = one_way;
      seg.code = lane.value("code", "");
      segs_.push_back(seg);
    }
  }
  return !segs_.empty() || root["lanes"].empty();
}

bool LaneAwareNavfn::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  return costmap_->worldToMap(wx, wy, mx, my);
}

void LaneAwareNavfn::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  costmap_->mapToWorld(mx, my, wx, wy);
}

bool LaneAwareNavfn::moveAllowed(double x0, double y0, double x1, double y1) const
{
  const double mx = 0.5 * (x0 + x1);
  const double my = 0.5 * (y0 + y1);
  const double move = std::atan2(y1 - y0, x1 - x0);

  const LaneSeg * best = nullptr;
  double best_d = std::numeric_limits<double>::infinity();
  for (const auto & seg : segs_) {
    double t = 0.0;
    const double d = distPointSeg(mx, my, seg.ax, seg.ay, seg.bx, seg.by, t);
    if (d <= seg.half_width && d < best_d) {
      best_d = d;
      best = &seg;
    }
  }
  if (!best || !best->one_way) {
    return true;
  }
  // ONE_WAY:
  // - forward half-plane (along >= 0) allowed
  // - near-perpendicular crossing (|along| < 0.25 ≈ 75–105°) allowed so free-space
  //   goals can cross LTR/RTL without being forced around the east pinch
  // - reverse and reverse-diagonal (8-connected zigzag west on eastbound lane) forbidden
  const double err = normalizeAngle(move - best->tangent);
  const double along = std::cos(err);
  if (std::fabs(along) < 0.25) {
    return true;
  }
  return along >= -1e-9;
}

bool LaneAwareNavfn::onLaneCorridor(double wx, double wy) const
{
  for (const auto & seg : segs_) {
    double t = 0.0;
    if (distPointSeg(wx, wy, seg.ax, seg.ay, seg.bx, seg.by, t) <= seg.half_width) {
      return true;
    }
  }
  return false;
}

nav_msgs::msg::Path LaneAwareNavfn::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_.lock()->now();
  path.header.frame_id = global_frame_;

  unsigned int sx, sy, gx, gy;
  if (!worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
    !worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy))
  {
    RCLCPP_WARN(node_.lock()->get_logger(), "LaneAwareNavfn: start/goal off costmap");
    return path;
  }

  const unsigned int size_x = costmap_->getSizeInCellsX();
  const unsigned int size_y = costmap_->getSizeInCellsY();
  const auto idx = [size_x](unsigned int x, unsigned int y) {
      return y * size_x + x;
    };

  const unsigned int start_i = idx(sx, sy);
  const unsigned int goal_i = idx(gx, gy);
  const unsigned char sc = costmap_->getCost(sx, sy);
  const unsigned char gc = costmap_->getCost(gx, gy);
  // Distinguish NO_INFORMATION / LETHAL / INSCRIBED. Default start_goal threshold is
  // LETHAL(254); INSCRIBED(253) is allowed at endpoints so parking near walls can plan,
  // while traversal still rejects cost >= lethal_cost_ (253).
  if (sc == nav2_costmap_2d::NO_INFORMATION || gc == nav2_costmap_2d::NO_INFORMATION) {
    RCLCPP_WARN(
      node_.lock()->get_logger(),
      "LaneAwareNavfn: start/goal NO_INFORMATION (start=%u goal=%u)", sc, gc);
    return path;
  }
  if (sc >= static_cast<unsigned char>(start_goal_lethal_cost_) ||
    gc >= static_cast<unsigned char>(start_goal_lethal_cost_))
  {
    RCLCPP_WARN(
      node_.lock()->get_logger(),
      "LaneAwareNavfn: start/goal LETHAL (start=%u goal=%u threshold=%d)",
      sc, gc, start_goal_lethal_cost_);
    return path;
  }
  if (sc >= static_cast<unsigned char>(lethal_cost_) ||
    gc >= static_cast<unsigned char>(lethal_cost_))
  {
    RCLCPP_INFO(
      node_.lock()->get_logger(),
      "LaneAwareNavfn: start/goal INSCRIBED/high (start=%u goal=%u) — allowing plan seed",
      sc, gc);
  }

  struct Node
  {
    unsigned int i;
    double g;
    double f;
  };
  struct Cmp
  {
    bool operator()(const Node & a, const Node & b) const {return a.f > b.f;}
  };

  std::priority_queue<Node, std::vector<Node>, Cmp> open;
  std::unordered_map<unsigned int, double> gscore;
  std::unordered_map<unsigned int, unsigned int> parent;
  gscore[start_i] = 0.0;
  open.push(Node{start_i, 0.0, 0.0});

  static const int dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  static const int dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  static const double cost8[8] = {1, 1.41421356, 1, 1.41421356, 1, 1.41421356, 1, 1.41421356};

  bool found = false;
  unsigned int expansions = 0;
  while (!open.empty()) {
    if (cancel_checker && cancel_checker()) {
      RCLCPP_WARN(node_.lock()->get_logger(), "LaneAwareNavfn: cancelled");
      return path;
    }
    const Node cur = open.top();
    open.pop();
    if (++expansions > 2'000'000u) {
      RCLCPP_WARN(node_.lock()->get_logger(), "LaneAwareNavfn: expansion limit");
      return path;
    }
    if (cur.i == goal_i) {
      found = true;
      break;
    }
    if (gscore.count(cur.i) && cur.g > gscore[cur.i] + 1e-9) {
      continue;
    }
    const unsigned int cx = cur.i % size_x;
    const unsigned int cy = cur.i / size_x;
    double wx0, wy0;
    mapToWorld(cx, cy, wx0, wy0);

    for (int k = 0; k < 8; ++k) {
      const int nx = static_cast<int>(cx) + dx8[k];
      const int ny = static_cast<int>(cy) + dy8[k];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x) || ny >= static_cast<int>(size_y)) {
        continue;
      }
      const unsigned int ux = static_cast<unsigned int>(nx);
      const unsigned int uy = static_cast<unsigned int>(ny);
      const unsigned char c = costmap_->getCost(ux, uy);
      if (c >= static_cast<unsigned char>(lethal_cost_) || c == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }
      double wx1, wy1;
      mapToWorld(ux, uy, wx1, wy1);
      if (!moveAllowed(wx0, wy0, wx1, wy1)) {
        continue;
      }
      const unsigned int ni = idx(ux, uy);
      // Strongly prefer free cells so the search does not graze INSCRIBED corridors that
      // later become disconnected pockets under live inflation + ONE_WAY.
      // Evidence: leave EXIT still plan_min_y≈-0.89 with prefer=0.35 — inflated east aisle
      // step≈8.7 still beat free south step=1. On mapped lanes use weak inflation weight.
      const double cost_pen = static_cast<double>(c) / 255.0;
      double step;
      if (onLaneCorridor(wx1, wy1)) {
        step = cost8[k] * (1.0 + 2.0 * cost_pen * cost_pen) * lane_prefer_factor_;
      } else {
        step = cost8[k] * (1.0 + 12.0 * cost_pen * cost_pen);
      }
      const double ng = cur.g + step;
      if (!gscore.count(ni) || ng + 1e-9 < gscore[ni]) {
        gscore[ni] = ng;
        parent[ni] = cur.i;
        const double h = std::hypot(static_cast<double>(gx) - nx, static_cast<double>(gy) - ny);
        open.push(Node{ni, ng, ng + h});
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(node_.lock()->get_logger(), "LaneAwareNavfn: no path (ONE_WAY / lethal)");
    return path;
  }

  std::vector<unsigned int> rev;
  for (unsigned int i = goal_i; ; i = parent[i]) {
    rev.push_back(i);
    if (i == start_i) {
      break;
    }
  }
  std::reverse(rev.begin(), rev.end());

  for (unsigned int i : rev) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    const unsigned int mx = i % size_x;
    const unsigned int my = i / size_x;
    mapToWorld(mx, my, p.pose.position.x, p.pose.position.y);
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  // Orient each pose along the path tangent. Leaving w=1 (yaw=0) on every cell makes
  // Regulated Pure Pursuit report immediate "collision ahead" on skid-steer warehouse paths.
  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const double dx = path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x;
    const double dy = path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y;
    if (dx * dx + dy * dy < 1e-12) {
      continue;
    }
    const double yaw = std::atan2(dy, dx);
    path.poses[i].pose.orientation.z = std::sin(yaw * 0.5);
    path.poses[i].pose.orientation.w = std::cos(yaw * 0.5);
  }
  if (!path.poses.empty()) {
    path.poses.front().pose.position = start.pose.position;
    path.poses.front().header = path.header;
    // Keep tangent yaw on start when possible; fall back to start orientation.
    if (path.poses.size() == 1) {
      path.poses.front().pose.orientation = start.pose.orientation;
    }
    path.poses.back() = goal;
    path.poses.back().header = path.header;
  }
  return path;
}

}  // namespace warotrans_lane_planner

PLUGINLIB_EXPORT_CLASS(warotrans_lane_planner::LaneAwareNavfn, nav2_core::GlobalPlanner)
