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
  bool onLaneCorridor(double wx, double wy) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::vector<LaneSeg> segs_;
  double corridor_margin_{0.05};
  /// Multiply step cost when cell is inside a mapped lane corridor (prefer leave/lanes).
  double lane_prefer_factor_{0.15};
  /// Cost threshold that blocks *traversal* (Nav2 LETHAL=254). INSCRIBED is soft-cost only.
  int lethal_cost_{254};
  /// Cost threshold that blocks *start/goal acceptance* (Nav2 LETHAL=254).
  int start_goal_lethal_cost_{254};
};

}  // namespace warotrans_lane_planner
