#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace warotrans_lane_planner
{

struct LaneSeg
{
  double ax, ay, bx, by;
  double tangent;
  double half_width;
  bool one_way;
  std::string code;
};

struct WorldPoint
{
  double x, y;
};

class LaneAwareNavfn : public nav2_core::GlobalPlanner
{
public:
  LaneAwareNavfn() = default;
  ~LaneAwareNavfn() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  bool loadLanes(const std::string & path);
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;
  bool moveAllowed(double x0, double y0, double x1, double y1) const;
  /// Lane whose corridor contains the point and whose centre line is closest; nullptr when off-lane.
  const LaneSeg * nearestLane(double wx, double wy, double & distance) const;
  /// Cost of travelling dist_cells at this point (the search and path simplification share it).
  double stepCost(double wx, double wy, unsigned char cell_cost, double dist_cells) const;
  /// Cost of the straight segment a-b; false if it breaks a lane direction or enters a cell above max_cell_cost.
  bool segmentCost(
    const WorldPoint & a, const WorldPoint & b, unsigned char max_cell_cost, double & total) const;
  /// True when the segment runs along a lane away from its centre: costs alone allow that, since the wall inflation
  /// on one side of a lane can pay for the offset, and on opposite lanes it brings the two paths together.
  bool shortcutLeavesLaneCentre(const WorldPoint & a, const WorldPoint & b) const;
  /// Replaces grid staircases with straight segments that are no worse than the cells they replace.
  std::vector<WorldPoint> simplifyPath(const std::vector<WorldPoint> & points) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::vector<LaneSeg> segs_;
  double corridor_margin_{0.05};
  /// Multiply step cost when cell is inside a mapped lane corridor (prefer leave/lanes).
  double lane_prefer_factor_{0.15};
  /// Extra cost at the corridor edge relative to the lane centre line: 1 + w * (d / half_width)^2.
  double lane_center_weight_{3.0};
  double start_escape_radius_{0.30};
  /// Largest offset from the lane centre of a shortcut along a lane, away from its ends. Above half a costmap cell,
  /// or a centre line between two cells would forbid every shortcut.
  double lane_shortcut_max_offset_{0.035};
  /// Cost threshold that blocks *traversal* (Nav2 LETHAL=254). INSCRIBED is soft-cost only.
  int lethal_cost_{254};
  /// Cost threshold that blocks *start/goal acceptance* (Nav2 LETHAL=254).
  int start_goal_lethal_cost_{254};
};

}  // namespace warotrans_lane_planner
