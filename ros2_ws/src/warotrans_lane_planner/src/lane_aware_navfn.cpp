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

// A shortcut may cost this much more than the grid cells it replaces, so rounding does not keep staircases alive.
constexpr double kShortcutCostSlack = 1.02;
// Near either end of a shortcut the path may be joining or leaving the lane centre.
constexpr double kShortcutEndAllowance = 0.30;
// Shortcut heading within about 25 degrees of the lane: it runs along the lane rather than across it.
constexpr double kAlongLaneMinCos = 0.9;

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
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lane_center_weight", rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".start_escape_radius", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lane_shortcut_max_offset", rclcpp::ParameterValue(0.035));

  std::string lanes_path;
  node->get_parameter(name_ + ".lanes_json", lanes_path);
  node->get_parameter(name_ + ".corridor_margin", corridor_margin_);
  node->get_parameter(name_ + ".lethal_cost", lethal_cost_);
  node->get_parameter(name_ + ".start_goal_lethal_cost", start_goal_lethal_cost_);
  node->get_parameter(name_ + ".lane_prefer_factor", lane_prefer_factor_);
  node->get_parameter(name_ + ".lane_center_weight", lane_center_weight_);
  node->get_parameter(name_ + ".start_escape_radius", start_escape_radius_);
  node->get_parameter(name_ + ".lane_shortcut_max_offset", lane_shortcut_max_offset_);

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

  double best_d = 0.0;
  const LaneSeg * best = nearestLane(mx, my, best_d);
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

const LaneSeg * LaneAwareNavfn::nearestLane(double wx, double wy, double & distance) const
{
  const LaneSeg * best = nullptr;
  distance = std::numeric_limits<double>::infinity();
  for (const auto & seg : segs_) {
    double t = 0.0;
    const double d = distPointSeg(wx, wy, seg.ax, seg.ay, seg.bx, seg.by, t);
    if (d <= seg.half_width && d < distance) {
      distance = d;
      best = &seg;
    }
  }
  return best;
}

double LaneAwareNavfn::stepCost(double wx, double wy, unsigned char cell_cost, double dist_cells) const
{
  // Prefer free cells. Off-lane: strong inflation weight. On-lane: centre pull + elevated
  // INSCRIBED soft-cost so the planner does not squeeze through inflated pockets when a clear
  // lane centre exists (do NOT hard-block 253 — that historically erased mid-aisle routes).
  const double cost_pen = static_cast<double>(cell_cost) / 255.0;
  double distance = 0.0;
  const LaneSeg * lane = nearestLane(wx, wy, distance);
  if (!lane) {
    return dist_cells * (1.0 + 12.0 * cost_pen * cost_pen);
  }
  const double off_center = distance / lane->half_width;
  // INSCRIBED(253) and near-inscribed: heavier soft penalty inside mapped lanes.
  const double inscribed_boost =
    (cell_cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) ? 6.0 :
    (cell_cost >= 200) ? 3.0 : 1.0;
  return dist_cells * (1.0 + 2.0 * cost_pen * cost_pen * inscribed_boost) * lane_prefer_factor_ *
         (1.0 + lane_center_weight_ * off_center * off_center);
}

bool LaneAwareNavfn::segmentCost(
  const WorldPoint & a, const WorldPoint & b, unsigned char max_cell_cost, double & total) const
{
  const double resolution = costmap_->getResolution();
  const int samples = std::max(
    1, static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / (0.5 * resolution))));
  WorldPoint prev = a;
  total = 0.0;
  for (int i = 1; i <= samples; ++i) {
    const double t = static_cast<double>(i) / samples;
    const WorldPoint q{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)};
    unsigned int mx, my;
    if (!worldToMap(q.x, q.y, mx, my)) {
      return false;
    }
    const unsigned char c = costmap_->getCost(mx, my);
    if (c > max_cell_cost || c >= static_cast<unsigned char>(lethal_cost_) ||
      c == nav2_costmap_2d::NO_INFORMATION || !moveAllowed(prev.x, prev.y, q.x, q.y))
    {
      return false;
    }
    total += stepCost(q.x, q.y, c, std::hypot(q.x - prev.x, q.y - prev.y) / resolution);
    prev = q;
  }
  return true;
}

bool LaneAwareNavfn::shortcutLeavesLaneCentre(const WorldPoint & a, const WorldPoint & b) const
{
  const double length = std::hypot(b.x - a.x, b.y - a.y);
  if (length <= 2.0 * kShortcutEndAllowance) {
    return false;
  }
  const double heading = std::atan2(b.y - a.y, b.x - a.x);
  const int samples = static_cast<int>(std::ceil(length / costmap_->getResolution()));
  for (int i = 1; i < samples; ++i) {
    const double t = static_cast<double>(i) / samples;
    if (std::min(t, 1.0 - t) * length < kShortcutEndAllowance) {
      continue;
    }
    double distance = 0.0;
    const LaneSeg * lane = nearestLane(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), distance);
    if (lane && distance > lane_shortcut_max_offset_ &&
      std::fabs(std::cos(normalizeAngle(heading - lane->tangent))) > kAlongLaneMinCos)
    {
      return true;
    }
  }
  return false;
}

std::vector<WorldPoint> LaneAwareNavfn::simplifyPath(const std::vector<WorldPoint> & points) const
{
  const size_t n = points.size();
  if (n < 3) {
    return points;
  }
  const double resolution = costmap_->getResolution();
  std::vector<unsigned char> cell(n, 0);
  std::vector<double> cost_to(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    unsigned int mx, my;
    if (!worldToMap(points[i].x, points[i].y, mx, my)) {
      return points;
    }
    cell[i] = costmap_->getCost(mx, my);
    if (i > 0) {
      const double dist = std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
      cost_to[i] = cost_to[i - 1] + stepCost(points[i].x, points[i].y, cell[i], dist / resolution);
    }
  }

  std::vector<WorldPoint> out{points.front()};
  std::vector<unsigned char> max_cell(n, 0);
  size_t i = 0;
  while (i + 1 < n) {
    unsigned char highest = cell[i];
    for (size_t j = i + 1; j < n; ++j) {
      highest = std::max(highest, cell[j]);
      max_cell[j] = highest;
    }
    // The farthest point reachable without getting closer to an obstacle than the replaced cells.
    size_t next = i + 1;
    for (size_t j = n - 1; j > i + 1; --j) {
      double cost = 0.0;
      if (!shortcutLeavesLaneCentre(points[i], points[j]) &&
        segmentCost(points[i], points[j], max_cell[j], cost) &&
        cost <= (cost_to[j] - cost_to[i]) * kShortcutCostSlack)
      {
        next = j;
        break;
      }
    }
    out.push_back(points[next]);
    i = next;
  }
  return out;
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
  // The keep-out planning mask is wider than the keep-out itself and leaves only centimetres beside some lanes, so
  // localization noise can put the robot inside it. That is no reason to have no path: plan out of the lethal blob
  // the robot stands in, within start_escape_radius. The goal is never relaxed, and the footprint check of the
  // controller still decides whether the robot may actually move.
  const bool start_lethal = sc >= static_cast<unsigned char>(start_goal_lethal_cost_);
  const bool escaping = start_lethal && start_escape_radius_ > 0.0;
  if ((start_lethal && !escaping) || gc >= static_cast<unsigned char>(start_goal_lethal_cost_)) {
    RCLCPP_WARN(
      node_.lock()->get_logger(),
      "LaneAwareNavfn: start/goal LETHAL (start=%u goal=%u threshold=%d)",
      sc, gc, start_goal_lethal_cost_);
    return path;
  }
  if (escaping) {
    RCLCPP_WARN(
      node_.lock()->get_logger(),
      "LaneAwareNavfn: start is LETHAL (cost %u): planning out of it within %.2f m", sc, start_escape_radius_);
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
  // The cheapest possible cell is a lane centre. A larger heuristic would overestimate inside lanes,
  // turning A* greedy: non-optimal, zigzagging paths.
  const double heuristic_scale = segs_.empty() ? 1.0 : std::min(1.0, lane_prefer_factor_);

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
      if (c == nav2_costmap_2d::NO_INFORMATION) {
        continue;
      }
      double wx1, wy1;
      mapToWorld(ux, uy, wx1, wy1);
      if (c >= static_cast<unsigned char>(lethal_cost_)) {
        const bool leaving_start_blob =
          escaping && costmap_->getCost(cx, cy) >= static_cast<unsigned char>(lethal_cost_) &&
          std::hypot(wx1 - start.pose.position.x, wy1 - start.pose.position.y) <= start_escape_radius_;
        if (!leaving_start_blob) {
          continue;
        }
      }
      if (!moveAllowed(wx0, wy0, wx1, wy1)) {
        continue;
      }
      const unsigned int ni = idx(ux, uy);
      const double ng = cur.g + stepCost(wx1, wy1, c, cost8[k]);
      if (!gscore.count(ni) || ng + 1e-9 < gscore[ni]) {
        gscore[ni] = ng;
        parent[ni] = cur.i;
        const double h = std::hypot(static_cast<double>(gx) - nx, static_cast<double>(gy) - ny);
        open.push(Node{ni, ng, ng + heuristic_scale * h});
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

  std::vector<WorldPoint> cells;
  cells.reserve(rev.size() + 1);
  for (unsigned int i : rev) {
    WorldPoint p{};
    mapToWorld(i % size_x, i / size_x, p.x, p.y);
    cells.push_back(p);
  }
  cells.front() = WorldPoint{start.pose.position.x, start.pose.position.y};
  const WorldPoint goal_point{goal.pose.position.x, goal.pose.position.y};
  if (cells.size() == 1) {
    cells.push_back(goal_point);
  } else {
    cells.back() = goal_point;
  }
  const std::vector<WorldPoint> corners = simplifyPath(cells);

  // Back to one pose per cell length: the controller and the traffic reservation expect a dense path.
  const double resolution = costmap_->getResolution();
  for (size_t c = 0; c + 1 < corners.size(); ++c) {
    const WorldPoint & a = corners[c];
    const WorldPoint & b = corners[c + 1];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const int samples = std::max(1, static_cast<int>(std::ceil(length / resolution)));
    const double yaw = std::atan2(b.y - a.y, b.x - a.x);
    for (int s = 0; s < samples; ++s) {
      const double t = static_cast<double>(s) / samples;
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = a.x + t * (b.x - a.x);
      p.pose.position.y = a.y + t * (b.y - a.y);
      // Orient each pose along the path tangent. Leaving w=1 (yaw=0) on every cell makes
      // Regulated Pure Pursuit report immediate "collision ahead" on skid-steer warehouse paths.
      if (length > 1e-6) {
        p.pose.orientation.z = std::sin(yaw * 0.5);
        p.pose.orientation.w = std::cos(yaw * 0.5);
      } else {
        p.pose.orientation = start.pose.orientation;
      }
      path.poses.push_back(p);
    }
  }
  path.poses.push_back(goal);
  path.poses.back().header = path.header;
  return path;
}

}  // namespace warotrans_lane_planner

PLUGINLIB_EXPORT_CLASS(warotrans_lane_planner::LaneAwareNavfn, nav2_core::GlobalPlanner)
