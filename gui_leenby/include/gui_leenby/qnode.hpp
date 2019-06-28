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
#include <QImage>

namespace gui_leenby {

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
        bool init();
        void run();
        void changerCommande(double vx, double vy, double vz, double rx, double ry, double rz);
        void imageCallback(const sensor_msgs::ImageConstPtr& msg);
        QImage retour;

Q_SIGNALS:
        void rosShutdown();
        void hasReceivedImage(); // Envoyé dès qu'une image est reçue

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
        image_transport::Subscriber sub;
        geometry_msgs::Twist commande;
};

}

#endif /* gui_leenby_QNODE_HPP_ */
