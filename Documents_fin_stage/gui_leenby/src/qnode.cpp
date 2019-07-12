
#include "../include/gui_leenby/qnode.hpp"

#include <string>
#include <std_msgs/String.h>
#include <sstream>

#include <std_msgs/String.h>
#include <image_transport/image_transport.h>

namespace gui_leenby {

// Constructeur
QNode::QNode(int argc, char** argv ) : init_argc(argc), init_argv(argv)
{

}

// Destructeur
QNode::~QNode()
{
    if(ros::isStarted())
    {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

// Initialisation du thread
//TDO: Modifier ici les topics de communication
bool QNode::init()
{
    ros::init(init_argc,init_argv,"gui_leenby");
    if ( ! ros::master::check() )
    {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    chatter_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // Publisher pour le déplacement
    parolePublisher = n.advertise<std_msgs::String>("/messageToSay", 1); // Publisher pour la parole
    retourGauche = QImage(); // Initialisation du retour de la caméra gauche
    retourDroite = QImage(); // Initialisation du retour de la caméra droite
    image_transport::ImageTransport it(n);
    subGauche = it.subscribe("/camera/image", 1, &QNode::imageCallbackGauche, this); //TODO: modifier le nom du topic pour celui de la camera gauche
    subDroite = it.subscribe("/camera/image", 1, &QNode::imageCallbackDroite, this); //TODO: modifier le nom du topic pour celui de la camera droite
    manette_sub=n.subscribe<sensor_msgs::Joy>("/joy",10,&QNode::manetteCallback, this); // Subscriber pour la manette

    start();
	return true;
}

// Run du thread
void QNode::run()
{
    ros::Rate loop_rate(30);
    while ( ros::ok() )
    {
        chatter_publisher.publish(commande); // Publie la commande
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

// Fonction permettant de changer l'information de commande du mouvement
void QNode::changerCommande(double vx, double vy, double vz, double rx, double ry, double rz)
{
        commande.linear.x=vx;
        commande.linear.y=vy;
        commande.linear.z=vz;
        commande.angular.x=rx;
        commande.angular.y=ry;
        commande.angular.z=rz;
}

// Fonction callback du retour de la caméra gauche
void QNode::imageCallbackGauche(const sensor_msgs::ImageConstPtr& msg)
{
    retourGauche = QImage(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
    if( retourGauche.isNull() )
    {
        ROS_ERROR("No image data on left camera");
    }
    else
    {
        Q_EMIT QNode::hasReceivedImage();
    }
}

// Fonction callback du retour de la caméra droite
void QNode::imageCallbackDroite(const sensor_msgs::ImageConstPtr& msg)
{
    retourDroite = QImage(&(msg->data[0]), msg->width, msg->height, QImage::Format_RGB888);
    if( retourDroite.isNull() )
    {
        ROS_ERROR("No image data on right camera");
    }
    else
    {
        Q_EMIT QNode::hasReceivedImage();
    }
}

// Fonction callback de la manette physique
void QNode::manetteCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
     Q_EMIT QNode::manetteMoved(joy->axes[1],joy->axes[0]);
}

// Fonction publiant le message pour faire parler le robot
void QNode::publishParoleMsg(std::string input)
{
    std_msgs::String msg;
    msg.data = input;
    parolePublisher.publish(msg);
}

}
