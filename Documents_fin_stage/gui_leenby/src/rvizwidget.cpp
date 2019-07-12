#include <QVBoxLayout>

#include <rviz/visualization_manager.h>
#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/displays_panel.h>
#include <string>

#include "../include/gui_leenby/rvizwidget.hpp"

RvizWidget::RvizWidget( QWidget* parent ) : QWidget( parent )
{
    render_panel = new rviz::RenderPanel();
    rviz::DisplaysPanel* displays_panel = new rviz::DisplaysPanel();
    QHBoxLayout* main_layout = new QHBoxLayout;
    main_layout->addWidget( displays_panel );
    main_layout->addWidget( render_panel );
    manager = new rviz::VisualizationManager(render_panel);

    // Set the top-level layout for this LidarView widget.
    setLayout( main_layout );

    // Next we initialize the main RViz classes.
    //
    // The VisualizationManager is the container for Display objects,
    // holds the main Ogre scene, holds the ViewController, etc.  It is
    // very central and we will probably need one in every usage of
    // librviz.
    render_panel->initialize( manager->getSceneManager(), manager );
    render_panel->contextMenuVisible();
    render_panel->setMinimumSize(200,200);
    manager->initialize();
    manager->startUpdate();
    displays_panel->initialize(manager);
}

RvizWidget::~RvizWidget()
{
  delete manager;
}
