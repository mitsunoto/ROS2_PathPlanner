#pragma once

#include <memory>
#include <string>
#include <QMainWindow>
#include "path_controller/map_widget.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"

/**
 * Создает главное окно с виджетом MapWidget и панелью управления.
 * @param widget     ссылочный указатель на создаваемый MapWidget
 * @param width      ширина окна
 * @param height     высота окна
 * @param goal_client   клиент параметров для узла goal_handler
 * @param robot_client  клиент параметров для узла robot_controller
 * @return указатель на QMainWindow
 */
QMainWindow* createMainWindow(
  MapWidget*& widget,
  int width,
  int height,
  std::shared_ptr<rclcpp::SyncParametersClient> goal_param_client,
  std::shared_ptr<rclcpp::SyncParametersClient> robot_param_client
);