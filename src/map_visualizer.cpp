#include "path_controller/map_visualizer.hpp"
#include "path_controller/map_widget.hpp"
#include <QPainter>
#include <QPen>
#include <QDebug>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <filesystem>

MapVisualizer::MapVisualizer(int width, int height,
                             rclcpp::Logger logger,
                             MapWidget *widget)
  : window_width_(width),
    window_height_(height),
    logger_(logger),
    widget_(widget)
{
  // Инициализация пустого изображения
  mapImage_ = QImage(window_width_, window_height_, QImage::Format_RGB888);
  mapImage_.fill(Qt::white);

  // Загрузка ориентиров
  std::string pkg_share = ament_index_cpp::get_package_share_directory("path_controller");
  std::filesystem::path landmarks_file_path = std::filesystem::path(pkg_share) / "assets" / "skyscrapers.csv";
  landmarks_ = readLandmarksFromFile(landmarks_file_path.string());
  if (landmarks_.empty()) {
    RCLCPP_WARN(logger_, "No landmarks loaded");
  } else {
    RCLCPP_INFO(logger_, "Loaded %zu landmarks", landmarks_.size());
  }
}

void MapVisualizer::updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr &map_msg, const std::string &map_level)
{
  map_width_  = map_msg->info.width;
  map_height_ = map_msg->info.height;
  pixel_size_ = window_height_ / static_cast<double>(map_height_);

  // Вычисляем новый размер изображения с отступами
  int img_w = widget_->marginLeft() + static_cast<int>(map_width_ * pixel_size_)  + 1;
  int img_h = widget_->marginTop() + static_cast<int>(map_height_ * pixel_size_) + 1;

  mapImage_ = QImage(img_w, img_h, QImage::Format_RGB888);
  mapImage_.fill(Qt::white);
  QPainter painter(&mapImage_);

  // Сдвигаем координаты, чтобы рисовать саму карту внутри отступов
  painter.translate(widget_->marginLeft(), widget_->marginTop());

  // Рисуем клетки
  for (int y = 0; y < map_height_; ++y) {
    for (int x = 0; x < map_width_; ++x) {
      int value = map_msg->data[y * map_width_ + x];
      QColor color = colorForValue(value);
      QRectF cellRect(x * pixel_size_, y * pixel_size_, pixel_size_, pixel_size_);
      painter.fillRect(cellRect, color);
    }
  }

  // Рисуем сетку
  drawGrid(&painter, map_level);

  // Возвращаем координаты
  painter.translate(-widget_->marginLeft(), -widget_->marginTop());

  // И наконец — оси «снаружи» карты
  drawAxes(&painter, map_level);

  painter.end();
  widget_->updateImage(mapImage_);
  RCLCPP_INFO(logger_, "Map updated, size: %d x %d", map_width_, map_height_);
}

void MapVisualizer::updateFineMap(const nav_msgs::msg::OccupancyGrid::SharedPtr &map_msg)
{
  fine_map_width_ = map_msg->info.width;
  fine_map_height_ = map_msg->info.height;

  widget_->setMapWidth(fine_map_width_);
  widget_->setMapHeight(fine_map_height_);

  fine_pixel_size_ = window_height_ / static_cast<double>(fine_map_height_);
  RCLCPP_INFO(logger_, "Fine map updated: size %d x %d, pixel_size=%.2f",
    fine_map_width_, fine_map_height_, fine_pixel_size_);
}

void MapVisualizer::updateRobotPosition(const geometry_msgs::msg::PoseStamped::SharedPtr &pose_msg)
{
  int x = static_cast<int>(std::ceil(pose_msg->pose.position.x));
  int y = static_cast<int>(std::ceil(pose_msg->pose.position.y));
  RCLCPP_INFO(logger_, "Received robot pose: (%d, %d)", x, y);
  if (x >= 0 && x < fine_map_width_ && y >= 0 && y < fine_map_height_) {
    widget_->updateRobotPosition(x, y, fine_pixel_size_);
    RCLCPP_INFO(logger_, "Robot position updated on widget using fine map dimensions.");
  } else {
    RCLCPP_WARN(logger_, "Robot pose (%d, %d) is out of fine map bounds (%d, %d)",
                x, y, fine_map_width_, fine_map_height_);
  }
}

void MapVisualizer::updateGoalPosition(const geometry_msgs::msg::PoseStamped::SharedPtr &goal_msg)
{
  int x = static_cast<int>(std::round(goal_msg->pose.position.x));
  int y = static_cast<int>(std::round(goal_msg->pose.position.y));
  widget_->updateGoalPose(x, y, fine_pixel_size_);
}

void MapVisualizer::updateLandmarkLinesToRobot(const geometry_msgs::msg::PoseStamped::SharedPtr &est_pose)
{
  widget_->setEstimatedPose(est_pose->pose.position.x, est_pose->pose.position.y);

  std::vector<QLineF> lines;
  if (landmarks_.empty()) return;
  double rx = (est_pose->pose.position.x + 0.5) * fine_pixel_size_;
  double ry = (est_pose->pose.position.y + 0.5) * fine_pixel_size_;
  for (const auto &lm : landmarks_) {
    double lx = (lm.x + 0.5) * fine_pixel_size_;
    double ly = (lm.y + 0.5) * fine_pixel_size_;
    lines.push_back(QLineF(lx + widget_->marginLeft(),
                  ly + widget_->marginTop(),
                  rx + widget_->marginLeft(),
                  ry + widget_->marginTop())); 
      RCLCPP_INFO(logger_, "Drawing line from landmark %s at (%.2f, %.2f) to robot at (%.2f, %.2f)",
                lm.id.c_str(), lx, ly, rx, ry);
  }
  widget_->updateLandmarkLines(lines);
}

// Отрисовка запланированного маршрута на отдельном слое
// Этот метод вычисляет линии для маршрута и в зависимости от цвета (параметра pathColor)
// передаёт их в соответствующий слой: shortestPathGroup_ или safePathGroup_
void MapVisualizer::updatePaths(const nav_msgs::msg::Path::SharedPtr &paths_msg, const QColor &pathColor) {
  std::vector<QLineF> lines;
  QPoint prevPoint(-1, -1);
  size_t index = 0;

  RCLCPP_INFO(rclcpp::get_logger("MapVisualizer"), "updatePaths: received %zu poses", paths_msg->poses.size());

  for (const auto &pose : paths_msg->poses) {
    double x_real = pose.pose.position.x;
    double y_real = pose.pose.position.y;

    int x = static_cast<int>(std::round(x_real));
    int y = static_cast<int>(std::round(y_real));
  
    if (x < 0 || x >= fine_map_width_ || y < 0 || y >= fine_map_height_) {
      RCLCPP_WARN(rclcpp::get_logger("MapVisualizer"), "[%zu] Skipped: out of map bounds (map size: %d x %d)", index, fine_map_width_, fine_map_height_);
      ++index;
      continue;
    }

    int px = static_cast<int>(x * fine_pixel_size_ + fine_pixel_size_ / 2 + widget_->marginLeft());
    int py = static_cast<int>(y * fine_pixel_size_ + fine_pixel_size_ / 2 + widget_->marginTop());
    QPoint currPoint(px, py);

    if (prevPoint.x() != -1) {
      lines.push_back(QLineF(prevPoint, currPoint));
    }

    prevPoint = currPoint;
    ++index;
  }

  RCLCPP_INFO(rclcpp::get_logger("MapVisualizer"), "Total lines created: %zu", lines.size());

  // Выбираем слой по цвету маршрута
  if (pathColor == QColor(17, 209, 209)) { // shortest_path
    widget_->updateShortestPathLines(lines);
  } else if (pathColor == QColor(51, 155, 57)) { // safe_path
    widget_->updateSafePathLines(lines);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("MapVisualizer"), "Unknown path color, updating default shortest path lines.");
    widget_->updateShortestPathLines(lines);
  }
}

void MapVisualizer::setLandmarkAngles(const std::vector<double> &angles) {
  landmark_angles_ = angles;
}

void MapVisualizer::updateDisplay() {
  if (widget_) {
    widget_->updateImage(mapImage_);
  }
}

void MapVisualizer::clearAllPaths() {
  widget_->updateShortestPathLines({});
  widget_->updateSafePathLines({});
}

QColor MapVisualizer::colorForValue(int value) {
  if (value >= 20) {
    int height = value - 20;
    double factor = std::min(height / 10.0, 1.0) * 0.5;
    return QColor(
      std::max(0, static_cast<int>(244 * (1 - factor))),
      std::max(0, static_cast<int>(176 * (1 - factor))),
      std::max(0, static_cast<int>(131 * (1 - factor)))
    );
  } else if (value == 2) {
    return QColor(244,176,131);
  } else if (value == 3) {
    return QColor(50,108,209);
  } else if (value == 1) {
    return QColor(195,201,212);
  } else if (value == 0) {
    return Qt::white;
  } else {
    return Qt::black;
  }
}

void MapVisualizer::drawGrid(QPainter *painter, const std::string& map_level) {
  const int fine_per_medium = 5;
  const int fine_per_coarse = 25;

  auto drawLines = [&](int step, const QColor& color, int penWidth){
    QPen pen(color);
    pen.setWidth(penWidth);
    painter->setPen(pen);
    for (int y = 0; y <= map_height_; y += step) {
      painter->drawLine(0, y * pixel_size_, map_width_ * pixel_size_, y * pixel_size_);
    }
    for (int x = 0; x <= map_width_; x += step) {
      painter->drawLine(x * pixel_size_, 0, x * pixel_size_, map_height_ * pixel_size_);
    }
  };

  if (map_level == "fine") {
    drawLines(1, QColor("#868482"), 0.3);
    drawLines(fine_per_medium, Qt::black, 1);
    drawLines(fine_per_coarse, QColor("#35322f"), 2);
  } else if (map_level == "medium") {
    drawLines(1, QColor("#868482"), 1);
    drawLines(fine_per_medium, Qt::black, 3);
  } else {
    drawLines(1, Qt::black, 2);
  }
}

void MapVisualizer::drawAxes(QPainter *painter, const std::string& map_level) {
  int step;
  if (map_level == "fine") step = 25;
  else if (map_level == "medium") step = 5;
  else step = 1;

  QFont font = painter->font();
  font.setPointSize(20);
  painter->setFont(font);
  painter->setPen(Qt::black);

  QPen pen(Qt::black);
  pen.setWidth(map_level == "coarse" ? 3 : 2);
  painter->setPen(pen);

  int marginLeft = widget_->marginLeft(); 
  int marginTop = widget_->marginTop(); 
  QFontMetrics fm(font);

  RCLCPP_WARN(rclcpp::get_logger("MapVisualizer"), "pixel_size = %f", pixel_size_);

  // Ось X сверху (подписи по центру ячеек)
  for (int x = 0, label = 0; x < map_width_; x += step, ++label) {
    // реальный центр группы клеток
    double centerX = (x + step / 2.0) * pixel_size_;
    int px = marginLeft + static_cast<int>(centerX);
  
    // буква-метка вместо цифры
    QChar ch = QChar('A' + (label % 26));
    QString str(ch);
  
    // вычисляем ширину текста и корректируем, чтобы центрировать
    int textW = fm.horizontalAdvance(str);
  
    // Y‑координата над полосой (например, в середине верхнего отступа)
    int py = marginTop - marginTop / 2;
  
    painter->drawText(px - textW / 2, py, str);
}

  // Ось Y слева: подписи по центру клеток
  for (int y = 0, label = 0; y < map_height_; y += step, ++label) {
    // центр группы клеток по Y
    double centerY = (y + step / 2.0) * pixel_size_;
    // вычисляем положение базовой линии так, чтобы текст оказался вертикально выровнен по центру
    double baselineY = marginTop + centerY + (fm.ascent() - fm.descent()) / 2.0;

    QString str = QString::number(label);
    // ширина текста, чтобы сдвинуть его вправо или влево
    int textW = fm.horizontalAdvance(str);

    // Х‑координата линии оси Y (например, на половине левого отступа)
    int px = marginLeft - marginLeft / 2;
    // рисуем так, чтобы правый край текста лежал на px
    painter->drawText(px - textW, int(baselineY), str);
  }

  // Горизонтальные отметки (выступы сверху)
  for (int x = 0; x <= map_width_; x += step) {
    int px = marginLeft + static_cast<int>(x * pixel_size_);
    int py1 = marginTop - 100;
    int py2 = marginTop;
    painter->drawLine(px, py1, px, py2);
  }

  // Вертикальные отметки (выступы слева)
  for (int y = 0; y <= map_height_; y += step) {
    int py = marginTop + static_cast<int>(y * pixel_size_);
    int px1 = marginLeft - 100;
    int px2 = marginLeft;
    painter->drawLine(px1, py, px2, py);
  }
}