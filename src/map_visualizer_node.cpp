#include "path_controller/map_visualizer_node.hpp"
#include "path_controller/map_visualizer.hpp"
#include "path_controller/map_widget.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include <QColor>

// ======================
// MapVisualizerNode: ROS2-узел, который получает карты, позиции, углы,
// и обновляет отображение, включая отрисовку запланированных маршрутов на отдельных слоях.
// ======================
MapVisualizerNode::MapVisualizerNode(int width, int height, MapWidget *widget)
  : Node("map_visualizer_node")
{
  RCLCPP_INFO(get_logger(), "узел запущен");

  rclcpp::QoS latched_qos(10);
  latched_qos.transient_local();

  // Создаём визуализатор
  map_visualizer_ = std::make_shared<MapVisualizer>(width, height, get_logger(), widget);

  // Подписки на разные уровни карт
  fine_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map/fine", latched_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      fine_map_ = msg;
      map_visualizer_->updateFineMap(msg);
      if (current_level_ == "fine") {
        map_visualizer_->updateMap(msg, "fine");
      }
    });

  medium_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map/medium", latched_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      medium_map_ = msg;
      if (current_level_ == "medium") {
        map_visualizer_->updateMap(msg, "medium");
      }
    });

  coarse_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map/coarse", latched_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      coarse_map_ = msg;
      if (current_level_ == "coarse") {
        map_visualizer_->updateMap(msg, "coarse");
      }
    });

  // Подписка на текущую позицию робота
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "current_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      map_visualizer_->clearAllPaths();
      RCLCPP_INFO(get_logger(), "Получена текущая позиция робота");
      map_visualizer_->updateRobotPosition(msg);
    });

  // Подписка на целевую позицию
  goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      map_visualizer_->clearAllPaths();
      RCLCPP_INFO(get_logger(), "Получена цель: x=%.2f, y=%.2f", 
                  msg->pose.position.x, msg->pose.position.y);
      map_visualizer_->updateGoalPosition(msg);
    });

  // Подписка на готовые маршруты
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "paths", 10,
    [this](const nav_msgs::msg::Path::SharedPtr msg) {
      if (msg->header.frame_id == "shortest_path") {
        RCLCPP_INFO(get_logger(), "Получен кратчайший маршрут");
        map_visualizer_->updatePaths(msg, QColor(17, 209, 209));
      } else if (msg->header.frame_id == "safe_path") {
        RCLCPP_INFO(get_logger(), "Получен безопасный маршрут");
        map_visualizer_->updatePaths(msg, QColor(51, 155, 57));
      } else {
        RCLCPP_WARN(get_logger(), "Неизвестный тип маршрута");
      }
    });

  // Подписка на оценённую позицию (для линий от ориентиров)
  estimated_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "estimated_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      RCLCPP_INFO(get_logger(), "Получена оценённая позиция: x=%.2f, y=%.2f",
                  msg->pose.position.x, msg->pose.position.y);
      map_visualizer_->updateLandmarkLinesToRobot(msg);
    });

  // Углы ориентиров
  angles_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    "landmark_angles", 10,
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      RCLCPP_INFO(get_logger(), "Получены углы ориентиров: %zu элементов", msg->data.size());
      map_visualizer_->setLandmarkAngles(msg->data);
    });

  // Пустой таймер
  timer_ = create_wall_timer(
    std::chrono::milliseconds(100), []() {});
}

void MapVisualizerNode::onScaleChanged(double scale) {
  std::string new_level;
  if (scale >= 2.0) new_level = "fine";
  else if (scale >= 1.0) new_level = "medium";
  else new_level = "coarse";

  if (new_level == current_level_) return;

  current_level_ = new_level;
  RCLCPP_INFO(get_logger(), "Переключаем уровень детализации: %s", current_level_.c_str());

  if (current_level_ == "fine" && fine_map_) {
    map_visualizer_->updateMap(fine_map_, "fine");
  } else if (current_level_ == "medium" && medium_map_) {
    map_visualizer_->updateMap(medium_map_, "medium");
  } else if (current_level_ == "coarse" && coarse_map_) {
    map_visualizer_->updateMap(coarse_map_, "coarse");
  }
}
