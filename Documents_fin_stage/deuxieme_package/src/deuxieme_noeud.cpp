#include <stdio.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void premier_noeudCallback(const geometry_msgs::Twist::ConstPtr& msg){
    ROS_INFO("\n"
             "Commande envoyee a la tortue :\n"
             "  Lineaire :\n"
             "      X : [%f], Y : [%f], Z : [%f]\n"
             "  Angulaire :\n"
             "      X : [%f], Y : [%f], Z : [%f]\n",
             msg->linear.x, msg->linear.y, msg->linear.z,
             msg->angular.x, msg->angular.y, msg->angular.z);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "deuxieme_noeud");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/turtle1/cmd_vel",1000, premier_noeudCallback);

    ros::spin();
    return 0;
}
