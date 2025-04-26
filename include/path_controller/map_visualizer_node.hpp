#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class MapWidget;
class MapVisualizer;

// ======================
// MapVisualizerNode: ROS2-узел, который получает карты, позиции, углы и estimated_pose,
// и обновляет отображение, включая отрисовку запланированных маршрутов на отдельных слоях в Qt-виджете
// ======================
class MapVisualizerNode : public rclcpp::Node {
public:
  MapVisualizerNode(int width, int height, MapWidget *widget);

  /// Обработчик изменения масштаба в виджете
  void onScaleChanged(double scale);

private:
  /// Объект, отвечающий за отрисовку карты и маршрутов
  std::shared_ptr<MapVisualizer> map_visualizer_;

  // Подписки на топики
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr fine_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr medium_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr coarse_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr angles_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_sub_;

  // Таймер для периодических обновлений
  rclcpp::TimerBase::SharedPtr timer_;

  // Кэш последних сообщений
  nav_msgs::msg::OccupancyGrid::SharedPtr fine_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr medium_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr coarse_map_;

  // Текущий уровень детализации: "fine", "medium" или "coarse"
  std::string current_level_ = "medium";
};
