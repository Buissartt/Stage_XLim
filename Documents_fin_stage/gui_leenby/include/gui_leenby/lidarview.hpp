#ifndef LIDARVIEW_HPP
#define LIDARVIEW_HPP
#include <QWidget>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

class LidarView: public QWidget
{
Q_OBJECT
public:
  LidarView(std::string topic, QWidget* parent = 0 );
  virtual ~LidarView();

private:
  rviz::RenderPanel* render_panel;

  rviz::VisualizationManager* manager;

  rviz::Display* laser_tete;
  rviz::Display* laser_base;
};
#endif // LIDARVIEW_HPP

