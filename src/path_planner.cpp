#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

// Узел планирования пути на основе A* с учётом мобильных препятствий
class PathPlanner : public rclcpp::Node {
public:
  PathPlanner() : Node("path_planner")
  {
    RCLCPP_INFO(get_logger(), "узел запущен");

    // Подписка на текущую позицию робота
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "current_pose", 10,
      std::bind(&PathPlanner::poseCallback, this, std::placeholders::_1));

    // Подписка на карту
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map/fine", 10,
      std::bind(&PathPlanner::mapCallback, this, std::placeholders::_1));

    // Подписка на мобильные препятствия
    mobile_obstacles_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "mobile_obstacles", 10,
      std::bind(&PathPlanner::mobileObstaclesCallback, this, std::placeholders::_1));

    // Подписка на целевую позицию
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      std::bind(&PathPlanner::goalCallback, this, std::placeholders::_1));

    // Иницицализация топика для публикации путей
    path_pub_ = create_publisher<nav_msgs::msg::Path>("paths", 10);
  }

private:
  double goal_x_{0.0}, goal_y_{0.0};
  bool has_pose_{false}, has_map_{false}, has_goal_{false};
  geometry_msgs::msg::PoseStamped current_pose_;
  nav_msgs::msg::OccupancyGrid map_;
  int width_{0}, height_{0};
  std::vector<geometry_msgs::msg::Pose> mobile_obstacles_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr mobile_obstacles_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Обработчик текущей позиции
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_pose_ = *msg;
    has_pose_ = true;
    RCLCPP_INFO(get_logger(), "Получена текущая позиция");
    tryPlanPaths();
  }

  // Обработчик карты
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = *msg;
    width_ = map_.info.width;
    height_ = map_.info.height;
    has_map_ = true;
    RCLCPP_INFO(get_logger(), "Получена карта (%dx%d)", width_, height_);
    tryPlanPaths();
  }

  // Обработчик мобильных препятствий
  void mobileObstaclesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    mobile_obstacles_ = msg->poses;
    RCLCPP_INFO(get_logger(), "Получены %zu мобильных препятствий", mobile_obstacles_.size());
    tryPlanPaths();
  }

  // Обработчик целевой позиции
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    has_goal_ = true;
    RCLCPP_INFO(get_logger(), "Получена цель [x=%.2f, y=%.2f]", goal_x_, goal_y_);
    tryPlanPaths();
  }

  // Запуск планирования, если все данные готовы
  void tryPlanPaths()
  {
    if (!has_pose_ || !has_map_ || !has_goal_) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Запуск планирования пути");

    auto shortest = findPath(false);
    auto safe     = findPath(true);

    if (!shortest.poses.empty()) {
      shortest.header.stamp = now();
      shortest.header.frame_id = "shortest_path";
      path_pub_->publish(shortest);
      RCLCPP_INFO(get_logger(), "Опубликован кратчайший путь (%zu точек)", shortest.poses.size());
    }
    if (!safe.poses.empty()) {
      safe.header.stamp = now();
      safe.header.frame_id = "safe_path";
      path_pub_->publish(safe);
      RCLCPP_INFO(get_logger(), "Опубликован безопасный путь (%zu точек)", safe.poses.size());
    }
  }

  // A* поиск пути: avoid_danger = true для учёта пересечённой местности
  nav_msgs::msg::Path findPath(bool avoid_danger)
  {
    nav_msgs::msg::Path path;

    int start_x = static_cast<int>(current_pose_.pose.position.x);
    int start_y = static_cast<int>(current_pose_.pose.position.y);
    int goal_x = static_cast<int>(goal_x_);
    int goal_y = static_cast<int>(goal_y_);

    // Проверка границ
    if (start_x < 0 || start_x >= width_ || start_y < 0 || start_y >= height_ ||
        goal_x < 0 || goal_x >= width_ || goal_y < 0 || goal_y >= height_) {
      RCLCPP_ERROR(get_logger(), "Старт или цель вне карты");
      return path;
    }

    // Инициализация структур A*
    std::vector<std::vector<bool>> visited(height_, std::vector<bool>(width_, false));
    std::vector<std::vector<double>> cost(height_, std::vector<double>(width_, std::numeric_limits<double>::infinity()));
    std::vector<std::vector<std::pair<int,int>>> parent(height_, std::vector<std::pair<int,int>>(width_, {-1,-1}));
    std::vector<std::vector<int>> steps(height_, std::vector<int>(width_, std::numeric_limits<int>::max()));
    steps[start_y][start_x] = 0;

    // Функция определения эвристики
    auto heuristic = [&](int x, int y) {
      return std::hypot(goal_x - x, goal_y - y);
    };

    using Node = std::tuple<double,int,int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<>> pq;
    pq.emplace(0.0, start_x, start_y);
    cost[start_y][start_x] = 0.0;

    std::vector<std::pair<int,int>> directions = {{0,1},{1,0},{0,-1},{-1,0}};

    int step_count = 0;
    const int MAX_STEPS = width_ * height_;
    while (!pq.empty()) {
      auto [current_cost, x, y] = pq.top();
      pq.pop();

      if (visited[y][x]) continue;
      visited[y][x] = true;

      if (x == goal_x && y == goal_y) {
        // Строим путь
        while (x != start_x || y != start_y) {
          geometry_msgs::msg::PoseStamped pose;
          pose.pose.position.x = x;
          pose.pose.position.y = y;
          path.poses.push_back(pose);
          auto [px, py] = parent[y][x];
          x = px;
          y = py;
        }
        std::reverse(path.poses.begin(), path.poses.end());
        return path;
      }

      if (++step_count > MAX_STEPS) {
        RCLCPP_ERROR(this->get_logger(), "Exceeded maximum step count; path not found!");
        break;
      }

      for (auto [dx, dy] : directions) {
        int nx = x + dx;
        int ny = y + dy;
      
        if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) continue;
      
        int cell_value = map_.data[ny * width_ + nx];
        if (cell_value >= 20) continue;                // Статическое препятствие
        if (cell_value == 3) continue;                 // Динамическое препятствие
        if (avoid_danger && cell_value == 1) continue; // Пересеченная местность
        if (cell_value == -1) continue;                // Неизвестный объект
      
        // Длина пути на данный момент
        int path_steps_so_far = steps[y][x] + 1;

        // Обработка потенциальной зоны охвата мобильных препятствий
        if (avoid_danger) {
          bool is_dangerous = false;
          for (const auto& obs : mobile_obstacles_) {
            double obs_x = obs.position.x;
            double obs_y = obs.position.y;

            const double obstacle_speed_factor = 0.4;
            double distance = std::hypot(obs_x - nx, obs_y - ny);
            double max_obstacle_reach = path_steps_so_far * obstacle_speed_factor;
            
            if (distance <= max_obstacle_reach) {
              is_dangerous = true;
              break;
            }
          }
      
          if (is_dangerous) continue;
        }
      
        double additional_cost = (avoid_danger && cell_value == 1) ? 10.0 : 0.0;
        double new_cost = current_cost + 1.0 + additional_cost;
      
        if (new_cost < cost[ny][nx]) {
          cost[ny][nx] = new_cost;
          parent[ny][nx] = {x, y};
          pq.emplace(new_cost + heuristic(nx, ny), nx, ny);
      
          // Обновляем шаги только при улучшении пути
          steps[ny][nx] = path_steps_so_far;
        }
      }
    }

    RCLCPP_WARN(this->get_logger(), "Path not found!");
    return path;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanner>());
  rclcpp::shutdown();
  return 0;
}
