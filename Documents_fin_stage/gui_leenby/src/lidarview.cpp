#include <QVBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/render_panel.h"
#include "rviz/displays_panel.h"
#include <string>


#include "../include/gui_leenby/lidarview.hpp"

LidarView::LidarView( std::string topicToRead, QWidget* parent ) : QWidget( parent )
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

  // Create a laser display.
  laser_tete = manager->createDisplay( "rviz/LaserScan", "Laser scan tête", true );
  laser_base = manager->createDisplay( "rviz/LaserScan", "Laser scan base", true );

  // Setup des paramètres
  manager->setFixedFrame("laser");
  laser_tete->subProp("Topic")->setValue(topicToRead.c_str());

  laser_base->subProp("Size (Pixels)")->setValue("2"); laser_tete->subProp("Style")->setValue("Points");
  laser_tete->subProp("Size (Pixels)")->setValue("3");
  laser_tete->subProp("Alpha")->setValue("1");
  laser_tete->subProp("Decay Time")->setValue("0");
  laser_tete->subProp("Position Transformer")->setValue("XYZ");
  laser_tete->subProp("Color Transformer")->setValue("Intensity");
  laser_tete->subProp("Channel Name")->setValue("x");
  laser_tete->subProp("Invert Rainbow")->setValue("true");
  laser_tete->subProp("Autocompute Intensity Bounds")->setValue("true");

  /*laser_base->subProp("Topic")->setValue(topicToRead.c_str());
  laser_base->subProp("Style")->setValue("Points");
  laser_base->subProp("Alpha")->setValue("1");
  laser_base->subProp("Decay Time")->setValue("0");
  laser_base->subProp("Position Transformer")->setValue("XYZ");
  laser_base->subProp("Color Transformer")->setValue("Intensity");
  laser_base->subProp("Channel Name")->setValue("intensities");
  laser_base->subProp("Invert Rainbow")->setValue("true");
  laser_base->subProp("Autocompute Intensity Bounds")->setValue("true");*/
}

LidarView::~LidarView()
{
  delete manager;
}
