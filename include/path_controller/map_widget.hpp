
#pragma once

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsItemGroup>
#include <QLineF>
#include <QImage>
#include <QPoint>
#include <mutex>
#include <vector>

class MapWidget : public QGraphicsView {
  Q_OBJECT

public:
  explicit MapWidget(QWidget *parent = nullptr);

  int marginLeft() const;
  int marginTop() const;

  void setEstimatedPose(double x, double y);
  double estimatedX() const;
  double estimatedY() const;

  void setMapWidth(int width);
  void setMapHeight(int height);

  void updateImage(const QImage &img);
  void updateRobotPosition(int x, int y, double scale);
  void updateGoalPose(int x, int y, double scale);

  void updateLandmarkLines(const std::vector<QLineF> &lines);
  void setLandmarkLinesVisible(bool visible);

  void updateShortestPathLines(const std::vector<QLineF> &lines);
  void updateSafePathLines(const std::vector<QLineF> &lines);

  double currentScale() const;

signals:
  void scaleChanged(double scale);

protected:
  void wheelEvent(QWheelEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;

private:
  QGraphicsScene            *scene_;
  QGraphicsPixmapItem       *pixmapItem_;
  QGraphicsEllipseItem      *robotItem_;
  QGraphicsEllipseItem      *goalItem_;
  QGraphicsItemGroup        *landmarkLinesGroup_;
  QGraphicsItemGroup        *shortestPathGroup_;
  QGraphicsItemGroup        *safePathGroup_;
  QPixmap                    pixmap_;
  double                     scaleFactor_;
  bool                       isPanning_;
  QPoint                     panStartPoint_;
  std::mutex                 mutex_;
  double                     est_x_{0.0}, est_y_{0.0};
  int                        map_width_{0}, map_height_{0};

  // Отступы для осей
  static constexpr int       marginLeft_{50};
  static constexpr int       marginTop_{50};
};
