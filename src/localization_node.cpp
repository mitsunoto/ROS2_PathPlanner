#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// Структура для хранения ориентира (идентификатор + координаты + высота)
struct Landmark {
  std::string id;
  double x{0.0};
  double y{0.0};
  int height{0};
};

// Чтение ориентиров из CSV-файла (формат: id;x;y;height)
static std::vector<Landmark> readLandmarksFromFile(const std::string &filepath) {
  std::vector<Landmark> landmarks;
  std::ifstream file(filepath);
  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("LocalizationNode"),
                "Не удалось открыть файл ориентиров: %s", filepath.c_str());
    return landmarks;
  }

  std::string line;
  std::getline(file, line);  // пропускаем заголовок

  while (std::getline(file, line)) {
    std::istringstream ss(line);
    Landmark lm;
    std::string token;

    if (!std::getline(ss, token, ';')) continue;
    lm.id = token;
    if (!std::getline(ss, token, ';')) continue;
    lm.x = std::stod(token);
    if (!std::getline(ss, token, ';')) continue;
    lm.y = std::stod(token);
    if (!std::getline(ss, token, ';')) continue;
    lm.height = std::stoi(token);

    landmarks.push_back(lm);
  }

  return landmarks;
}

// Нормализация угла в диапазоне [-pi, +pi]
static double normalize_angle(double angle) {
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

// Узел локализации: по полученным углам ориентиров вычисляет позицию робота (метод триангуляции)
class LocalizationNode : public rclcpp::Node {
public:
  LocalizationNode() : Node("localization_node") {
    RCLCPP_INFO(this->get_logger(), "узел запущен");

    // Получаем путь к файлу ориентиров
    std::string pkg_share =  ament_index_cpp::get_package_share_directory("path_controller");
    landmarks_file_path_ = std::filesystem::path(pkg_share) / "assets" / "skyscrapers.csv";

    // Загружаем ориентиры
    landmarks_ = readLandmarksFromFile(landmarks_file_path_.string());

    if (landmarks_.size() < 2) {
      RCLCPP_ERROR(get_logger(),
                  "Недостаточно ориентиров для локализации (%zu)",
                  landmarks_.size());
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Загружено ориентиров: %zu", landmarks_.size());

    // Инициализация топика для публикации оцененной позиции робота
    estimated_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "estimated_pose", 10);

    // Подписка на топик с углами ориентиров
    angles_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "landmark_angles", 10,
        std::bind(&LocalizationNode::anglesCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "LocalizationNode запущен");
  }

private:
  std::vector<Landmark> landmarks_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr angles_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_pub_;
  std::filesystem::path landmarks_file_path_;

  // Обработчик входящих углов: триангуляция и публикация позиции
  void anglesCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    const auto &angles = msg->data;
    RCLCPP_INFO(get_logger(),
                "Получены углы для %zu ориентиров", angles.size());
    if (angles.size() < 2) {
        RCLCPP_WARN(get_logger(),
                    "Для триангуляции нужно хотя бы 2 ориентира");
        return;
    }

    // Запуск триангуляции
    const auto [x, y] = triangulate(angles);
    RCLCPP_INFO(get_logger(),
                "Оценка позиции робота: x=%.2f, y=%.2f", x, y);

    // Публикуем оценённую позицию
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = "estimated_pose";
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.orientation.w = 1.0;
    estimated_pose_pub_->publish(pose_msg);
    RCLCPP_INFO(this->get_logger(), "Опубликована оцененная позиция: [x: %.2f, y: %.2f]",
              pose_msg.pose.position.x, pose_msg.pose.position.y);
  }

  // Функция триангуляции для двух ориентиров.
  // measured_angles[i] — угол (в радианах) от робота до i-го ориентира.
  // Для вычисления линии от ориентира к роботу используем measured_angle + pi.
  std::pair<double,double> triangulate(const std::vector<double>& theta) {
    if (theta.size() < 2) {
      RCLCPP_ERROR(this->get_logger(), "Not enough angles for triangulation");
      return {0.0, 0.0};
    }
    
    // Перебираем все пары ориентиров, пока не найдем подходящую
    for (size_t i = 0; i < theta.size(); ++i) {
      for (size_t j = i + 1; j < theta.size(); ++j) {
        Landmark lm1 = landmarks_[i];
        Landmark lm2 = landmarks_[j];
        
        // Тангенсы измеренных углов
        double t1 = std::tan(theta[0]);
        double t2 = std::tan(theta[1]);

        // Координаты ориентиров
        double x1 = lm1.x; double y1 = lm1.y;
        double x2 = lm2.x; double y2 = lm2.y;
        
        // Проверка на численную устойчивость
        if (std::fabs(denominator) < 1e-6) {
          RCLCPP_WARN(this->get_logger(), "Ориентиры %s и %s лежат на параллельных прямых. Проверка следующей пары.",
                      lm1.id.c_str(), lm2.id.c_str());
          continue;
        }

        double x = (y2 - y1 + x1*t1 - x2*t2) / denominator;
        double y = y1 + t1 * (x - x1);

        return {x, y};
      }
    }
    
    RCLCPP_ERROR(this->get_logger(), "Линии от ориентиров параллельны, нельзя триангулировать");
    return {0.0, 0.0};
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
