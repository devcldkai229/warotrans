#ifndef WAROTRANS_FLEET_LAYER__FLEET_LAYER_HPP_
#define WAROTRANS_FLEET_LAYER__FLEET_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace warotrans_fleet_layer
{

/**
 * The other robots as lethal cells. The robot gateway publishes them (from the backend's fleet/poses) as points filling
 * each other robot's footprint plus a margin, in the map frame. This layer marks the points of the newest cloud and
 * keeps nothing between updates: a robot that drove away leaves no trace (no raytrace clearing needed), and a cloud
 * older than max_age is ignored (the gateway also sends empty clouds). The inflation layer after it inflates them.
 *
 * Why: a 2D lidar scanning ~18 cm above the floor sees little of a robot whose body is 10 cm high; two real robots came
 * within 5 cm of each other (2026-11-22 / 2026-11-23).
 */
class FleetLayer : public nav2_costmap_2d::Layer
{
public:
  FleetLayer() = default;
  ~FleetLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override {return false;}

private:
  struct Point
  {
    double x;
    double y;
  };

  void onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  std::mutex mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_;
  rclcpp::Time received_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Clock::SharedPtr clock_;

  std::vector<Point> points_;  // newest cloud, in the costmap's global frame
  bool had_bounds_{false};
  double last_min_x_{0}, last_min_y_{0}, last_max_x_{0}, last_max_y_{0};
  double max_age_{1.0};
  std::string topic_;
};

}  // namespace warotrans_fleet_layer

#endif  // WAROTRANS_FLEET_LAYER__FLEET_LAYER_HPP_
