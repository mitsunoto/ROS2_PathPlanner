#include "path_controller/map_widget.hpp"
#include <QGraphicsLineItem>
#include <QScrollBar>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QDebug>

MapWidget::MapWidget(QWidget *parent)
  : QGraphicsView(parent),
    scaleFactor_(1.0),
    isPanning_(false)
{
  scene_ = new QGraphicsScene(this);
  setScene(scene_);

  pixmapItem_ = new QGraphicsPixmapItem();
  scene_->addItem(pixmapItem_);

  // Робот
  robotItem_ = new QGraphicsEllipseItem(-5, -5, 10, 10);
  robotItem_->setBrush(Qt::red);
  robotItem_->setPen(Qt::NoPen);
  robotItem_->setZValue(100);
  scene_->addItem(robotItem_);
  robotItem_->hide();

  // Цель
  goalItem_ = new QGraphicsEllipseItem(-5, -5, 10, 10);
  goalItem_->setBrush(Qt::green);
  goalItem_->setPen(Qt::NoPen);
  goalItem_->setZValue(100);
  scene_->addItem(goalItem_);
  goalItem_->hide();

  // Слои для линий
  landmarkLinesGroup_ = new QGraphicsItemGroup();
  landmarkLinesGroup_->setZValue(90);
  scene_->addItem(landmarkLinesGroup_);
  landmarkLinesGroup_->hide();

  shortestPathGroup_ = new QGraphicsItemGroup();
  shortestPathGroup_->setZValue(80);
  scene_->addItem(shortestPathGroup_);

  safePathGroup_ = new QGraphicsItemGroup();
  safePathGroup_->setZValue(80);
  scene_->addItem(safePathGroup_);
}

int MapWidget::marginLeft() const   { return marginLeft_; }
int MapWidget::marginTop()  const   { return marginTop_; }

void MapWidget::setEstimatedPose(double x, double y) {
  est_x_ = x;
  est_y_ = y;
}
double MapWidget::estimatedX() const { return est_x_; }
double MapWidget::estimatedY() const { return est_y_; }

void MapWidget::setMapWidth(int width)   { map_width_ = width; }
void MapWidget::setMapHeight(int height) { map_height_ = height; }

void MapWidget::updateImage(const QImage &img) {
  std::lock_guard<std::mutex> lock(mutex_);
  pixmap_ = QPixmap::fromImage(img);
  pixmapItem_->setPixmap(pixmap_);
  scene_->setSceneRect(pixmap_.rect());
  qInfo() << "MapWidget: updateImage(): size =" << pixmap_.size();
}

void MapWidget::updateRobotPosition(int x, int y, double scale) {
  double posX = x * scale + scale / 2.0 + marginLeft_;
  double posY = y * scale + scale / 2.0 + marginTop_;
  qInfo() << "MapWidget: robot at" << posX << posY;
  robotItem_->setPos(posX, posY);
  robotItem_->show();
}

void MapWidget::updateGoalPose(int x, int y, double scale) {
  double posX = x * scale + scale / 2.0 + marginLeft_;
  double posY = y * scale + scale / 2.0 + marginTop_;
  qInfo() << "MapWidget: goal at" << posX << posY;
  goalItem_->setPos(posX, posY);
  goalItem_->show();
}

void MapWidget::updateLandmarkLines(const std::vector<QLineF> &lines) {
  for (auto *item : landmarkLinesGroup_->childItems()) {
    landmarkLinesGroup_->removeFromGroup(item);
    delete item;
  }
  QPen pen(Qt::red);
  pen.setWidth(2);
  pen.setStyle(Qt::DashLine);
  for (auto &line : lines) {
    auto *li = new QGraphicsLineItem(line);
    li->setPen(pen);
    landmarkLinesGroup_->addToGroup(li);
  }
  qInfo() << "MapWidget: landmark lines =" << lines.size();
}

void MapWidget::setLandmarkLinesVisible(bool visible) {
  landmarkLinesGroup_->setVisible(visible);
}

void MapWidget::updateShortestPathLines(const std::vector<QLineF> &lines) {
  for (auto *item : shortestPathGroup_->childItems()) {
    shortestPathGroup_->removeFromGroup(item);
    delete item;
  }
  QPen pen(Qt::blue);
  pen.setWidth(2);
  pen.setStyle(Qt::DashLine);
  for (auto &line : lines) {
    auto *li = new QGraphicsLineItem(line);
    li->setPen(pen);
    shortestPathGroup_->addToGroup(li);
  }
  qInfo() << "MapWidget: shortest path lines =" << lines.size();
}

void MapWidget::updateSafePathLines(const std::vector<QLineF> &lines) {
  for (auto *item : safePathGroup_->childItems()) {
    safePathGroup_->removeFromGroup(item);
    delete item;
  }
  QPen pen(Qt::green);
  pen.setWidth(2);
  pen.setStyle(Qt::DashLine);
  for (auto &line : lines) {
    auto *li = new QGraphicsLineItem(line);
    li->setPen(pen);
    safePathGroup_->addToGroup(li);
  }
  qInfo() << "MapWidget: safe path lines =" << lines.size();
}

double MapWidget::currentScale() const {
  return scaleFactor_;
}

void MapWidget::wheelEvent(QWheelEvent *event) {
  constexpr double zoomFactor = 1.15;
  if (event->angleDelta().y() > 0) {
    scale(zoomFactor, zoomFactor);
    scaleFactor_ *= zoomFactor;
  } else {
    scale(1.0/zoomFactor, 1.0/zoomFactor);
    scaleFactor_ /= zoomFactor;
  }
  emit scaleChanged(scaleFactor_);
  event->accept();
}

void MapWidget::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    isPanning_ = true;
    panStartPoint_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
  }
  QGraphicsView::mousePressEvent(event);
}

void MapWidget::mouseMoveEvent(QMouseEvent *event) {
  if (isPanning_) {
    QPointF delta = mapToScene(event->pos()) - mapToScene(panStartPoint_);
    panStartPoint_ = event->pos();
    horizontalScrollBar()->setValue(horizontalScrollBar()->value() - delta.x());
    verticalScrollBar()->setValue(verticalScrollBar()->value() - delta.y());
  }
  QGraphicsView::mouseMoveEvent(event);
}

void MapWidget::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    isPanning_ = false;
    setCursor(Qt::ArrowCursor);
  }
  QGraphicsView::mouseReleaseEvent(event);
}
