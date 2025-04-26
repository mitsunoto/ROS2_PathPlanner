#pragma once

#include <QImage>
#include <QColor>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "path_controller/landmark_utils.hpp"

class MapWidget;

// ======================
// MapVisualizer: строит QImage по OccupancyGrid и обновляет MapWidget
// ======================
class MapVisualizer {
public:
  MapVisualizer(int width, int height, rclcpp::Logger logger, MapWidget *widget);

  // Обновление изображение карты заданного уровня детализации
  void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr &map_msg, const std::string &level);

  // Сохранение fine-map размеров и расчёт масштаба
  void updateFineMap(const nav_msgs::msg::OccupancyGrid::SharedPtr &map_msg);

  // Обновление положения робота и цели
  void updateRobotPosition(const geometry_msgs::msg::PoseStamped::SharedPtr &pose_msg);
  void updateGoalPosition(const geometry_msgs::msg::PoseStamped::SharedPtr &goal_msg);

  // Линии от ориентиров к роботу
  void updateLandmarkLinesToRobot(const geometry_msgs::msg::PoseStamped::SharedPtr &est_pose);

  // Обновление маршрутов
  void updatePaths(const nav_msgs::msg::Path::SharedPtr &paths_msg, const QColor &path_color);

  // Сохранение углов ориентиров (при необходимости)
  void setLandmarkAngles(const std::vector<double> &angles);

  void updateDisplay();

  // Сброс отображения маршрутов
  void clearAllPaths();

private:
  int window_width_, window_height_;
  int map_width_{0}, map_height_{0};
  double pixel_size_{0.0};
  QImage mapImage_;
  rclcpp::Logger logger_;
  MapWidget *widget_{nullptr};

  // Fine-map параметры
  int fine_map_width_{0}, fine_map_height_{0};
  double fine_pixel_size_;

  std::vector<Landmark> landmarks_;
  std::vector<double> landmark_angles_;

  // Вспомогательные методы для отрисовки
  QColor colorForValue(int value);
  void drawGrid(QPainter *painter, const std::string &map_level);
  void drawAxes(QPainter *painter, const std::string &map_level);
};