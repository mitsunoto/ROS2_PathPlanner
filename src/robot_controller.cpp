#include <signal.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using std::placeholders::_1;

class RobotController : public rclcpp::Node
{
public:
  RobotController() : Node("robot_controller")
  {
    RCLCPP_INFO(this->get_logger(), "узел запущен");

    // Получаем путь к файлу ориентиров
    std::string pkg_share =  ament_index_cpp::get_package_share_directory("path_controller");
    landmarks_file_path_ = std::filesystem::path(pkg_share) / "assets" / "skyscrapers.csv";

    // Регистрируем параметры
    this->declare_parameter<double>("robot_x", 0.0);
    this->declare_parameter<double>("robot_y", 0.0);
    this->declare_parameter<bool>("clear", false);

    // Инициализируем топики для публикации текущего местоположения и углов ориентиров
    current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
    landmark_angles_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("landmark_angles", 10);

    // Регистрируем callback на изменение параметров
    param_cb_handle_ = this->add_on_set_parameters_callback(std::bind(&RobotController::onParametersSet, this, _1)
    );
  }

private:
  // Метод для обработки изменения параметров
  rcl_interfaces::msg::SetParametersResult onParametersSet(const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &p : params) {
      if (p.get_name() == "robot_x") {
        robot_x_ = p.as_double();
        x_set_ = true;
      }
      else if (p.get_name() == "robot_y") {
        robot_y_ = p.as_double();
        y_set_ = true;
      }
      else if (p.get_name() == "clear" && p.as_bool()) {
        robot_x_ = robot_y_ = 0.0;
        x_set_ = y_set_ = false;
      }
    }

    if (x_set_ && y_set_) {
      publishCurrentPose();
      loadAndPublishLandmarkAngles(landmarks_file_path_.string());
    }

    return result;
  }

  // Публикация текущего местоположения
  void publishCurrentPose()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "current_pose";
    msg.pose.position.x = robot_x_;
    msg.pose.position.y = robot_y_;
    current_pose_pub_->publish(msg);
    RCLCPP_INFO(get_logger(),
                "Published robot pose: x=%.2f, y=%.2f",
                robot_x_, robot_y_);
  }

  // Чтение ориентиров и публикация углов
  void loadAndPublishLandmarkAngles(const std::string &file_path)
  {
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open skyscrapers.csv");
      return;
    }
    std::string line;
    std::getline(file, line);  // пропускаем заголовок

    std_msgs::msg::Float64MultiArray angles_msg;
    while (std::getline(file, line)) {
      std::istringstream ss(line);
      std::string id, xs, ys, hs;
      if (!std::getline(ss, id, ';') ||
          !std::getline(ss, xs, ';') ||
          !std::getline(ss, ys, ';') ||
          !std::getline(ss, hs, ';'))
      {
        continue;
      }
      double x_lm = std::stod(xs);
      double y_lm = std::stod(ys);
      double dx = x_lm - robot_x_;
      double dy = y_lm - robot_y_;
      double angle = std::atan2(dy, dx);
      angles_msg.data.push_back(angle);
      RCLCPP_INFO(get_logger(),
                  "Landmark %s: angle=%.2f rad",
                  id.c_str(), angle);
    }

    landmark_angles_pub_->publish(angles_msg);
    RCLCPP_INFO(get_logger(),
                "Published %zu landmark angles",
                angles_msg.data.size());
  }

  // Параметры и флаги
  double robot_x_{0.0}, robot_y_{0.0};
  bool x_set_{false}, y_set_{false};

  // Путь к файлу ориентиров
  std::filesystem::path landmarks_file_path_;

  // ROS-сущности
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr landmark_angles_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

// Обработчик сигнала для корректного выключения
static void signal_handler(int)
{
  if (rclcpp::ok()) rclcpp::shutdown();
}

int main(int argc, char** argv)
{
  signal(SIGTERM, signal_handler);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotController>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
