#include <QVBoxLayout>

#include <rviz/visualization_manager.h>
#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <string>

#include "../include/rviz_widget/rvizwidget.hpp"

RvizWidget::RvizWidget( QWidget* parent ) : QWidget( parent )
{
  render_panel = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget( render_panel );

  // Set the top-level layout for this LidarView widget.
  setLayout( main_layout );

  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc.  It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager = new rviz::VisualizationManager( render_panel );
  render_panel->initialize( manager->getSceneManager(), manager );
  render_panel->setMinimumSize(200,400);
  manager->initialize();
  manager->startUpdate();

}

RvizWidget::~RvizWidget()
{
  delete manager;
}
