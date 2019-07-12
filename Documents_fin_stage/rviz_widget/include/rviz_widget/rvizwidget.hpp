#ifndef RVIZWIDGET_HPP
#define RVIZWIDGET_HPP
#include <QWidget>

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

class RvizWidget: public QWidget
{
Q_OBJECT
public:
  RvizWidget( QWidget* parent = 0 );
  virtual ~RvizWidget();

private:
  rviz::VisualizationManager* manager;
  rviz::RenderPanel* render_panel;
};
#endif // RVIZWIDGET_HPP

