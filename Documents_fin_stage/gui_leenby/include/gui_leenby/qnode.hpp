#ifndef gui_leenby_QNODE_HPP_
#define gui_leenby_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
#include <image_transport/image_transport.h>
#endif

#include <string>
#include <QThread>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <QImage>
#include <std_msgs/String.h>

namespace gui_leenby {

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
        bool init();
        void run();
        void changerCommande(double vx, double vy, double vz, double rx, double ry, double rz);
        void imageCallbackGauche(const sensor_msgs::ImageConstPtr& msg);
        void imageCallbackDroite(const sensor_msgs::ImageConstPtr& msg);
        void manetteCallback(const sensor_msgs::Joy::ConstPtr& joy);
        QImage retourGauche, retourDroite;
        void publishParoleMsg(std::string string);

Q_SIGNALS:
        void rosShutdown();
        void hasReceivedImage(); // Envoyé dès qu'une image est reçue
        void manetteMoved(double x,double y);

private:
	int init_argc;
	char** init_argv;
        ros::Publisher chatter_publisher, parolePublisher;
        image_transport::Subscriber subGauche, subDroite;
        geometry_msgs::Twist commande;
        ros::Subscriber manette_sub;
};

}

#endif /* gui_leenby_QNODE_HPP_ */
