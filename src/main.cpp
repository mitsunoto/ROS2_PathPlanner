#include <signal.h>
#include <unistd.h>
#include <QApplication>
#include <QProcess>
#include <QString>
#include <QTimer>
#include <QMainWindow>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QToolBar>
#include <QToolButton>
#include <QDialog>
#include <QDebug>
#include <QDialogButtonBox>
#include <QLineEdit>
#include <QLabel>
#include <QRegularExpressionValidator>
#include <QRegularExpression>

#include "path_controller/map_widget.hpp"
#include "path_controller/map_visualizer_node.hpp"
#include "path_controller/gui.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/parameter_client.hpp"

// Глобальные указатели на процессы
QProcess* robotProc = nullptr; // robot_controller
QProcess* goalProc = nullptr; // goal_handler
QProcess* pathPlannerProc = nullptr; // path_planner
QProcess* mapLoaderProc = nullptr; // mapLoader
QProcess* localizationProc = nullptr; // localization_node

// Функция запускает ROS2-процесс и возвращает указатель на QProcess
static QProcess* startProcess(const QString& package, const QString& exe) {
  QProcess* proc = new QProcess();
  proc->start("ros2", {"run", package, exe});
  if (!proc->waitForStarted(2000)) {
      qWarning() << "Не удалось запустить" << exe << ":" << proc->errorString();
  }
  return proc;
}

// Функция корректно завершает и удаляет процесс
static void stopProcess(QProcess*& proc) {
  if (!proc) return;

  // 1) Сначала «имитируем» Ctrl+C для ROS2-узла:
  ::kill(proc->processId(), SIGINT);
  if (!proc->waitForFinished(3000)) {
    // 2) Если не вышло — шлём SIGTERM
    proc->terminate();
    if (!proc->waitForFinished(2000)) {
      // 3) А если и это не помогло — SIGKILL
      proc->kill();
      proc->waitForFinished(500);
    }
  }

  delete proc;
  proc = nullptr;
}

int main(int argc, char** argv) {
  // 0) Запускаем внешние узлы
  mapLoaderProc    = startProcess("path_controller", "map_loader");
  mapLoaderProc->waitForStarted();
  //usleep(1500);
  goalProc         = startProcess("path_controller", "goal_handler");
  robotProc        = startProcess("path_controller", "robot_controller");
  pathPlannerProc  = startProcess("path_controller", "path_planner");
  pathPlannerProc->waitForStarted();
  localizationProc = startProcess("path_controller", "localization_node");

  // 1) Запуск Qt
  QApplication app(argc, argv);

  // 2) Инициализируем ROS2
  rclcpp::init(argc, argv);

  // создаём клиенты для параметров узлов goal_handler и robot_controller
  auto qt_goal_node  = std::make_shared<rclcpp::Node>("qt_goal_param_client");
  auto goal_client   = std::make_shared<rclcpp::SyncParametersClient>(qt_goal_node, "goal_handler");
  auto qt_robot_node = std::make_shared<rclcpp::Node>("qt_robot_param_client");
  auto robot_client  = std::make_shared<rclcpp::SyncParametersClient>(qt_robot_node, "robot_controller");

  // 3) Создаём окно визуализации
  int w=1000, h=700;
  MapWidget* widget = nullptr;
  createMainWindow(widget, w, h, goal_client, robot_client);
  auto map_node   = std::make_shared<MapVisualizerNode>(w, h, widget);

  QObject::connect(
    widget, &MapWidget::scaleChanged,
    [map_node](double newScale) {
        map_node->onScaleChanged(newScale);
    }
  );
  // 4) Спиннер только для MapVisualizerNode
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(map_node);
  QTimer rosTimer;
  QObject::connect(&rosTimer, &QTimer::timeout, [&](){ exec.spin_some(); });
  rosTimer.start(10);

  QObject::connect(&app, &QApplication::aboutToQuit, []() {
    stopProcess(goalProc);
    stopProcess(robotProc);
    stopProcess(pathPlannerProc);
    stopProcess(mapLoaderProc);
    stopProcess(localizationProc);
    rclcpp::shutdown();
  });

  // 5) Старт Qt цикла
  int ret = app.exec();

  // 6) Завершаем внешние процессы
  stopProcess(goalProc);
  stopProcess(robotProc);
  stopProcess(pathPlannerProc);
  stopProcess(mapLoaderProc);
  stopProcess(localizationProc);

  // 7) Завершаем ROS
  rclcpp::shutdown();
  return ret;
}
