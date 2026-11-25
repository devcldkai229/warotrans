#include "warotrans_fleet_layer/fleet_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace warotrans_fleet_layer
{

void FleetLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("FleetLayer: lifecycle node gone");
  }
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/fleet_obstacles")));
  declareParameter("max_age", rclcpp::ParameterValue(1.0));
  // The cost the other robots are drawn with, and how far around each of their points. LETHAL (254) is inflated by the
  // inflation layer (0.35 m on our maps), so the planner drove wide around a robot still far away (lab 2026-11-25).
  // INSCRIBED (253) grown by our own inscribed radius keeps the bodies apart without that halo.
  declareParameter("mark_cost", rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));
  declareParameter("grow_m", rclcpp::ParameterValue(0.0));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".topic", topic_);
  node->get_parameter(name_ + ".max_age", max_age_);
  int mark_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
  node->get_parameter(name_ + ".mark_cost", mark_cost);
  mark_cost_ = static_cast<unsigned char>(std::clamp(mark_cost, 1, static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));
  node->get_parameter(name_ + ".grow_m", grow_m_);
  grow_m_ = std::max(0.0, grow_m_);
  clock_ = node->get_clock();
  received_ = clock_->now();
  sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_, rclcpp::SensorDataQoS(),
    std::bind(&FleetLayer::onCloud, this, std::placeholders::_1));
  current_ = true;
  RCLCPP_INFO(
    node->get_logger(), "FleetLayer %s: other robots from %s (max age %.1f s, cost %d, grown %.2f m)", name_.c_str(),
    topic_.c_str(), max_age_, static_cast<int>(mark_cost_), grow_m_);
}

void FleetLayer::onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = msg;
  received_ = clock_->now();
}

void FleetLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_ && (clock_->now() - received_).seconds() <= max_age_) {
      cloud = latest_;
    }
  }

  points_.clear();
  if (cloud && cloud->width * cloud->height > 0) {
    const auto global = layered_costmap_->getGlobalFrameID();
    tf2::Transform to_global;
    to_global.setIdentity();
    bool ok = true;
    if (!cloud->header.frame_id.empty() && cloud->header.frame_id != global) {
      try {
        const auto t = tf_->lookupTransform(global, cloud->header.frame_id, tf2::TimePointZero);
        to_global.setOrigin(tf2::Vector3(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
        to_global.setRotation(
          tf2::Quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w));
      } catch (const tf2::TransformException &) {
        ok = false;  // no transform yet: draw nothing this time rather than in the wrong place
      }
    }
    if (ok) {
      sensor_msgs::PointCloud2ConstIterator<float> ix(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iy(*cloud, "y");
      for (; ix != ix.end(); ++ix, ++iy) {
        const tf2::Vector3 p = to_global * tf2::Vector3(*ix, *iy, 0.0);
        points_.push_back({p.x(), p.y()});
      }
    }
  }

  // Bounds: where the robots are now AND where they were at the last update, so the old marks are redrawn away.
  double nmin_x = std::numeric_limits<double>::max(), nmin_y = std::numeric_limits<double>::max();
  double nmax_x = std::numeric_limits<double>::lowest(), nmax_y = std::numeric_limits<double>::lowest();
  for (const auto & p : points_) {
    nmin_x = std::min(nmin_x, p.x - grow_m_);
    nmin_y = std::min(nmin_y, p.y - grow_m_);
    nmax_x = std::max(nmax_x, p.x + grow_m_);
    nmax_y = std::max(nmax_y, p.y + grow_m_);
  }
  if (had_bounds_) {
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
  }
  if (!points_.empty()) {
    *min_x = std::min(*min_x, nmin_x);
    *min_y = std::min(*min_y, nmin_y);
    *max_x = std::max(*max_x, nmax_x);
    *max_y = std::max(*max_y, nmax_y);
    last_min_x_ = nmin_x;
    last_min_y_ = nmin_y;
    last_max_x_ = nmax_x;
    last_max_y_ = nmax_y;
    had_bounds_ = true;
  } else {
    had_bounds_ = false;
  }
}

void FleetLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  const double res = master_grid.getResolution();
  const int reach = res > 0.0 ? static_cast<int>(std::ceil(grow_m_ / res)) : 0;
  const double reach2 = grow_m_ * grow_m_ + 1e-9;
  for (const auto & p : points_) {
    unsigned int mx = 0, my = 0;
    if (!master_grid.worldToMap(p.x, p.y, mx, my)) {
      continue;
    }
    for (int dj = -reach; dj <= reach; ++dj) {
      for (int di = -reach; di <= reach; ++di) {
        if ((di * res) * (di * res) + (dj * res) * (dj * res) > reach2) {
          continue;
        }
        const int i = static_cast<int>(mx) + di, j = static_cast<int>(my) + dj;
        if (i < min_i || i >= max_i || j < min_j || j >= max_j) {
          continue;
        }
        // Never lower what another layer marked (a wall stays a wall), and treat unknown as free.
        const unsigned char old = master_grid.getCost(i, j);
        if (old == nav2_costmap_2d::NO_INFORMATION || old < mark_cost_) {
          master_grid.setCost(i, j, mark_cost_);
        }
      }
    }
  }
}

void FleetLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.reset();
  points_.clear();
  had_bounds_ = false;
  current_ = true;
}

}  // namespace warotrans_fleet_layer

PLUGINLIB_EXPORT_CLASS(warotrans_fleet_layer::FleetLayer, nav2_costmap_2d::Layer)
