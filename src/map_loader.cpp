#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

class MapLoader : public rclcpp::Node {
public:
  MapLoader() : Node("map_loader") {
    RCLCPP_INFO(this->get_logger(), "узел запущен");

    rclcpp::QoS latched_qos(10);
    latched_qos.transient_local();

    // Инициализация топиков для публикации трех уровней детализации карты
    fine_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map/fine", latched_qos);
    medium_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map/medium", latched_qos);
    coarse_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map/coarse", latched_qos);

    // Инициализация топика для публикации мобильных препятствий
    mobile_obstacles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("mobile_obstacles", latched_qos);

    // Получение пути к изображению карты из папки ресурсов проекта
    std::string pkg_share = ament_index_cpp::get_package_share_directory("path_controller");
    std::filesystem::path map_path = std::filesystem::path(pkg_share) / "assets" / "initial_map.bmp";

    // Загрузка и публикация карт в топики
    loadMap(map_path.string());
  }

private:
  // Функция агрегации:
  // принимает вектор значений дочерних ячеек и возвращает тип самого
  // часто встречаемого препятствия, если препятствий более 50%, иначе 0.
  int aggregateCells(const std::vector<int> &cells) {
    int obstacle_count = 0;
    int difficult_terrain_count = 0;
    int max_height = 0;

    for (auto val : cells) {
      if (val == 1) {
        difficult_terrain_count++;
      }
      else if (val == 2 || val == 3) {
        obstacle_count++;
      }
      else if (val >= 20) {
        obstacle_count++;
        max_height = std::max(max_height, val-20);
      }
    }

    int total_count = obstacle_count + difficult_terrain_count;
    if (total_count > static_cast<int>(cells.size() / 2.5))
      return obstacle_count > difficult_terrain_count ? std::max(2,20+max_height) : 1;
    else
      return 0;
  }

    // Загрузка изображения и публикация трех уровней OccupancyGrid
  void loadMap(const std::string &file_path) {
    RCLCPP_INFO(this->get_logger(), "Загрузка карты: %s", file_path.c_str());
    cv::Mat image = cv::imread(file_path, cv::IMREAD_COLOR);
    if (image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Не удалось загрузить изображение карты");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Карта загружена: %dx%d", image.cols, image.rows);

    // 1. Формирование мелкой (детальной) сетки
    int fine_cell_size = 10;
    int fine_grid_width = image.cols / fine_cell_size;
    int fine_grid_height = image.rows / fine_cell_size;
    std::vector<int> fine_grid(fine_grid_width * fine_grid_height, 0);

    // Для мобильных препятствий
    geometry_msgs::msg::PoseArray mobile_obstacles;
    mobile_obstacles.header.frame_id = "map";

    // Определяем список коричневых оттенков (BGR, т.к. OpenCV)
    // Порядок: от светлого к темному. Используем индекс для определения высоты: height = (index+1), максимум 10.
    std::vector<cv::Vec3b> brownColors = {
      cv::Vec3b(131, 176, 244), // (244, 176, 131) -> height = 1
      cv::Vec3b(124, 167, 231), // (231, 167, 124) -> height = 2
      cv::Vec3b(117, 158, 219), // (219, 158, 117) -> height = 3
      cv::Vec3b(111, 149, 207), // (207, 149, 111) -> height = 4
      cv::Vec3b(104, 140, 195), // (195, 140, 104) -> height = 5
      cv::Vec3b(98, 132, 183),  // (183, 132, 98)  -> height = 6
      cv::Vec3b(91, 123, 170),  // (170, 123, 91)  -> height = 7
      cv::Vec3b(85, 114, 158),  // (158, 114, 85)  -> height = 8
      cv::Vec3b(78, 105, 146),  // (146, 105, 78)  -> height = 9
      cv::Vec3b(72, 96, 134),   // (134, 96, 72)   -> height = 10
      cv::Vec3b(65, 88, 122)    // (122, 88, 65)   -> height = 10
    };

      // Лямбда для проверки, является ли цвет одним из коричневых оттенков
    auto isBrownish = [&brownColors](const cv::Vec3b &color) -> bool {
      for (const auto &brown : brownColors) {
        if (color == brown)
          return true;
      }
      return false;
    };

      // Заполнение мелкой сетки по цветам (сопоставление цвета с проходимостью)
    for (int cell_y = 0; cell_y < fine_grid_height; ++cell_y) {
      for (int cell_x = 0; cell_x < fine_grid_width; ++cell_x) {
        int pixel_x = cell_x * fine_cell_size + fine_cell_size / 2;
        int pixel_y = cell_y * fine_cell_size + fine_cell_size / 2;
        cv::Vec3b color = image.at<cv::Vec3b>(pixel_y, pixel_x);
        int index = cell_y * fine_grid_width + cell_x;

        if (isBrownish(color)) {
          int obstacle_height = 0;
          for (size_t i = 0; i < brownColors.size(); ++i) {
            if (color == brownColors[i]) {
              obstacle_height = (i < 9 ? static_cast<int>(i + 1) : 10);
              break;
            }
          }
          // Кодируем высоту препятствия как 20 + obstacle_height
          fine_grid[index] = 20 + obstacle_height;
          // RCLCPP_INFO(this->get_logger(), "Cell (%d,%d) is brown, height=%d", cell_x, cell_y, obstacle_height);
        } else if (color == cv::Vec3b(209, 108, 50)) {
          fine_grid[index] = 3; // мобильное препятствие
          geometry_msgs::msg::Pose obstacle_pose;
          obstacle_pose.position.x = cell_x;
          obstacle_pose.position.y = cell_y;
          mobile_obstacles.poses.push_back(obstacle_pose);
        } else if (color == cv::Vec3b(212, 201, 195)) {
          fine_grid[index] = 1; // пересеченная местность
        } else if (color == cv::Vec3b(255, 255, 255)) {
          fine_grid[index] = 0; // свободная местность
        } else {
          RCLCPP_WARN(this->get_logger(), "Unrecognized color: (%d, %d, %d) at cell (%d,%d)",
                      color[0], color[1], color[2], cell_x, cell_y);
          fine_grid[index] = -1;
        }
      }
    }

    // 2. Формирование средней сетки (каждая ячейка = 5x5 мелких)
    int medium_factor = 5;
    int medium_grid_width = fine_grid_width / medium_factor;
    int medium_grid_height = fine_grid_height / medium_factor;
    std::vector<int> medium_grid(medium_grid_width * medium_grid_height, 0);
    for (int my = 0; my < medium_grid_height; ++my) {
      for (int mx = 0; mx < medium_grid_width; ++mx) {
        std::vector<int> children;
        for (int y = 0; y < medium_factor; ++y) {
          for (int x = 0; x < medium_factor; ++x) {
            int fine_x = mx * medium_factor + x;
            int fine_y = my * medium_factor + y;
            int index = fine_y * fine_grid_width + fine_x;
            children.push_back(fine_grid[index]);
          }
        }
        medium_grid[my * medium_grid_width + mx] = aggregateCells(children);
      }
    }

    // 3. Формирование крупной сетки (каждая ячейка = 5x5 средних)
    int coarse_factor = 5;
    int coarse_grid_width = medium_grid_width / coarse_factor;
    int coarse_grid_height = medium_grid_height / coarse_factor;
    std::vector<int> coarse_grid(coarse_grid_width * coarse_grid_height, 0);
    for (int cy = 0; cy < coarse_grid_height; ++cy) {
      for (int cx = 0; cx < coarse_grid_width; ++cx) {
        std::vector<int> children;
        for (int y = 0; y < coarse_factor; ++y) {
          for (int x = 0; x < coarse_factor; ++x) {
            int medium_x = cx * coarse_factor + x;
            int medium_y = cy * coarse_factor + y;
            int index = medium_y * medium_grid_width + medium_x;
            children.push_back(medium_grid[index]);
          }
        }
        coarse_grid[cy * coarse_grid_width + cx] = aggregateCells(children);
      }
    }

    // 4. Формирование и публикация сообщений OccupancyGrid для каждого уровня

    // Мелкая карта
    nav_msgs::msg::OccupancyGrid fine_map_msg;
    fine_map_msg.header.frame_id = "map/fine";
    fine_map_msg.header.stamp = rclcpp::Clock().now();
    fine_map_msg.info.width = fine_grid_width;
    fine_map_msg.info.height = fine_grid_height;
    fine_map_msg.info.origin.position.x = 0;
    fine_map_msg.info.origin.position.y = 0;
    fine_map_msg.data.resize(fine_grid_width * fine_grid_height);
    for (size_t i = 0; i < fine_grid.size(); ++i) {
      fine_map_msg.data[i] = fine_grid[i];
    }
    fine_map_pub_->publish(fine_map_msg);
    RCLCPP_INFO(this->get_logger(), "map/fine опубликована успешно (%dx%d)", fine_map_msg.info.width, fine_map_msg.info.height);

    // Средняя карта
    nav_msgs::msg::OccupancyGrid medium_map_msg;
    medium_map_msg.header.frame_id = "map/medium";
    medium_map_msg.header.stamp = rclcpp::Clock().now();
    medium_map_msg.info.width = medium_grid_width;
    medium_map_msg.info.height = medium_grid_height;
    medium_map_msg.info.origin.position.x = 0;
    medium_map_msg.info.origin.position.y = 0;
    medium_map_msg.data.resize(medium_grid_width * medium_grid_height);
    for (size_t i = 0; i < medium_grid.size(); ++i) {
      medium_map_msg.data[i] = medium_grid[i];
    }
    medium_map_pub_->publish(medium_map_msg);
    RCLCPP_INFO(this->get_logger(), "map/medium опубликована успешно (%dx%d)", medium_map_msg.info.width, medium_map_msg.info.height);

    // Крупная карта
    nav_msgs::msg::OccupancyGrid coarse_map_msg;
    coarse_map_msg.header.frame_id = "map/coarse";
    coarse_map_msg.header.stamp = rclcpp::Clock().now();
    coarse_map_msg.info.width = coarse_grid_width;
    coarse_map_msg.info.height = coarse_grid_height;
    coarse_map_msg.info.origin.position.x = 0;
    coarse_map_msg.info.origin.position.y = 0;
    coarse_map_msg.data.resize(coarse_grid_width * coarse_grid_height);
    for (size_t i = 0; i < coarse_grid.size(); ++i) {
      coarse_map_msg.data[i] = coarse_grid[i];
    }
    coarse_map_pub_->publish(coarse_map_msg);
    RCLCPP_INFO(this->get_logger(), "map/coarse опубликована успешно (%dx%d)", coarse_map_msg.info.width, coarse_map_msg.info.height);

    // Публикуем мобильные препятствия (из мелкого уровня)
    mobile_obstacles.header.stamp = rclcpp::Clock().now();
    mobile_obstacles_pub_->publish(mobile_obstacles);
    RCLCPP_INFO(this->get_logger(), "Мобильные препятствия опубликованы успешно");
  }

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr fine_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr medium_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr coarse_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr mobile_obstacles_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapLoader>());
  rclcpp::shutdown();
  return 0;
}
