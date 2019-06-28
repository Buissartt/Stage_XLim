
#include "../include/gui_leenby/qnode.hpp"

#include <string>
#include <std_msgs/String.h>
#include <sstream>

#include <image_transport/image_transport.h>

namespace gui_leenby {

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"gui_leenby");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	// Add your ros communications here.
    chatter_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    retour = QImage(); // Initialisation du retour caméra
    image_transport::ImageTransport it(n);
    sub = it.subscribe("/camera/image", 1, &QNode::imageCallback, this);

    start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(30);
	while ( ros::ok() ) {
        chatter_publisher.publish(commande); // Publie la commande
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::changerCommande(double vx, double vy, double vz, double rx, double ry, double rz){
        commande.linear.x=vx;
        commande.linear.y=vy;
        commande.linear.z=vz;
        commande.angular.x=rx;
        commande.angular.y=ry;
        commande.angular.z=rz;
}

void QNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    retour = QImage(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
    if( retour.isNull() )
    {
        ROS_ERROR("No image data");
    }
    else
    {
        Q_EMIT QNode::hasReceivedImage();
    }
}


}
