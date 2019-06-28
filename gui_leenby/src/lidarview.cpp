#include <QVBoxLayout>

#include <rviz/visualization_manager.h>
#include "rviz/display.h"
#include "rviz/render_panel.h"
#include <string>

#include "../include/gui_leenby/lidarview.hpp"

LidarView::LidarView( std::string topicToRead, QWidget* parent ) : QWidget( parent )
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

  // Create a laser display.
  laser_tete = manager->createDisplay( "rviz/LaserScan", "Laser scan tête", true );
  ROS_ASSERT( laser_tete != NULL ); // Vérifie que le display n'est pas vide
  laser_base = manager->createDisplay( "rviz/LaserScan", "Laser scan base", true );
  ROS_ASSERT( laser_base != NULL ); // Vérifie que le display n'est pas vide

  // Setup des paramètres
  manager->setFixedFrame("laser");
  laser_tete->subProp("Topic")->setValue(topicToRead.c_str());
  laser_tete->subProp("Style")->setValue("Points");
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
  laser_base->subProp("Size (Pixels)")->setValue("2");
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
