#include <QtGui>
#include <QApplication>
#include "../include/rviz_widget/rvizwidget.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    if(!ros::isInitialized())
    {
        ros::init(argc,argv, "rviz_widget");
    }
    QApplication app(argc, argv);
    RvizWidget* viewer = new RvizWidget();
    viewer->show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
