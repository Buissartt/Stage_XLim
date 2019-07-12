#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>

int main(int argc, char ** argv){
    ros::init(argc, argv,"premier_noeud");
    ros::NodeHandle n;

    ros::Rate loop_rate(5);

    geometry_msgs::Twist commande;

    ros::Publisher pub_commande=n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1);

    char key;
    while(ros::ok()){
        std::cin >> key;

        if(key == 'z'){
            commande.linear.x=2.0;
            commande.linear.y=0;
            commande.linear.z=0;
            commande.angular.x=0;
            commande.angular.y=0;
            commande.angular.z=0;

        } else if(key == 'q'){
            commande.linear.x=0;
            commande.linear.y=0;
            commande.linear.z=0;
            commande.angular.x=0;
            commande.angular.y=0;
            commande.angular.z=2.0;

        } else if(key == 's'){
            commande.linear.x=-2.0;
            commande.linear.y=0;
            commande.linear.z=0;
            commande.angular.x=0;
            commande.angular.y=0;
            commande.angular.z=0;

        } else if(key == 'd'){
            commande.linear.x=0;
            commande.linear.y=0;
            commande.linear.z=0;
            commande.angular.x=0;
            commande.angular.y=0;
            commande.angular.z=-2.0;

        }
        else {
            commande.linear.x=0;
            commande.linear.y=0;
            commande.linear.z=0;
            commande.angular.x=0;
            commande.angular.y=0;
            commande.angular.z=0;
            ROS_WARN("Touche non prise en charge");
        }

        pub_commande.publish(commande);
        loop_rate.sleep();
    }
}
