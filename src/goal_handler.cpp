#include <signal.h>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class GoalHandler : public rclcpp::Node
{
public:
  GoalHandler()
  : Node("goal_handler")
  {
    // Регистрация параметров
    this->declare_parameter<double>("goal_x", 0.0);
    this->declare_parameter<double>("goal_y", 0.0);
    this->declare_parameter<bool>("clear", false);

    // Инициализациця топика для публикации целевой позиции
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

    // Регистрация callback-функции на изменение параметров
    param_callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          if (p.get_name() == "goal_x") {
            goal_x_ = p.as_double();
            goal_x_set_ = true;
          }
          else if (p.get_name() == "goal_y") {
            goal_y_ = p.as_double();
            goal_y_set_ = true;
          }
          else if (p.get_name() == "clear" && p.as_bool()) {
            // Reset goals to zero
            goal_x_ = 0.0;
            goal_y_ = 0.0;
            goal_x_set_ = false;
            goal_y_set_ = false;
            // Reset the clear flag back to false
            // this->set_parameter(rclcpp::Parameter("clear", false));
          }
        }

        // Публикация по возможности
        if (goal_x_set_ && goal_y_set_) {
          publishGoal();
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      }
    );
  }

private:
  void publishGoal()
  {
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = goal_x_;
    msg.pose.position.y = goal_y_;
    goal_pub_->publish(msg);
    RCLCPP_INFO(get_logger(),
                "Published new goal: x=%.2f, y=%.2f",
                goal_x_, goal_y_);
  }

  double goal_x_{0.0}, goal_y_{0.0};
  bool goal_x_set_{false}, goal_y_set_{false};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

static void signal_handler(int)
{
  if (rclcpp::ok()) rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  signal(SIGTERM, signal_handler);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalHandler>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
