#ifndef LIDARVIEW_HPP
#define LIDARVIEW_HPP
#include <QWidget>
#include <rviz/render_panel.h>
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
  rviz::VisualizationManager* manager;
  rviz::RenderPanel* render_panel;
  rviz::Display* laser_tete;
  rviz::Display* laser_base;
};
#endif // LIDARVIEW_HPP

