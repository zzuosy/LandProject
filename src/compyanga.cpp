#include "ros/ros.h"
#include "std_msgs/String.h"
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "aruco.h"
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include<landing/pose.h>
#include<landing/arucopose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <gazebo_msgs/ModelStates.h>

using namespace std;

landing::pose pose;
void posecb(const landing::pose::ConstPtr& msg)
{
    pose = *msg;
}

landing::arucopose arucopose;
void arucocb(const landing::arucopose::ConstPtr& msg)
{
    arucopose = *msg;
}

gazebo_msgs::ModelStates models_state;
geometry_msgs::Pose uwb1;
geometry_msgs::Pose uwb2;
void uwb_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
    models_state = *msg;
    uwb1 = models_state.pose[2];
    uwb2 = models_state.pose[1];
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "compyanga_node");	//初始化ROS，节点命名为node_b，节点名必须唯一。
    ros::NodeHandle n;	//节点句柄实例化
    image_transport::ImageTransport it(n);

    ros::Rate rate(20.0);

    ros::Subscriber pose_sub = n.subscribe<landing::pose>("/pose_yolo",1,posecb);
    ros::Subscriber aruco_sub = n.subscribe<landing::arucopose>("/pose_aruco",1,arucocb);
    //ros::Subscriber uwb_sub = n.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 1, uwb_cb);

    double dx;
    double dy;
    double dyaw;

    double roll1;
    double roll2;
    double pitch1;
    double pitch2;
    double yaw1;
    double yaw2;

    ofstream outfile;
    //outfile.open("compare.csv",ios::out);
    //outfile<<"time"<<','<<"tx"<<','<<"ty"<<','<<"tyaw"<<','<<"yx"<<','<<"yy"<<','<<"yyaw"<<','<<"ax"<<','<<"ay"<<','<<"ayaw"<<endl;
    outfile.open("difflight.csv",ios::out);
    outfile<<"time"<<','<<"yx"<<','<<"yy"<<','<<"ax"<<','<<"ay"<<','<<endl;

    ros::Time start = ros::Time::now();
    while(ros::ok())
    {   
        //dx = uwb1.position.x - uwb2.position.x;
        //dy = uwb1.position.y - uwb2.position.y;
        //tf::Quaternion quat1;
        //tf::Quaternion quat2;
        //tf::quaternionMsgToTF(uwb1.orientation, quat1);
        //tf::quaternionMsgToTF(uwb2.orientation, quat2);      
        //tf::Matrix3x3(quat1).getRPY(roll1, pitch1, yaw1);
        //tf::Matrix3x3(quat2).getRPY(roll2, pitch2, yaw2);
        //dyaw = yaw1 - yaw2;
        double yx = pose.posestamped.pose.position.x;
        double yy = pose.posestamped.pose.position.y;
        //double yyaw = pose.yaw;
        double ax = arucopose.pointstamped.point.x;
        double ay = arucopose.pointstamped.point.y;
        //double ayaw = arucopose.yaw;
        double secs =(ros::Time::now() - start).toSec();
        outfile<<secs<<','<<yx<<','<<yy<<','<<ax<<','<<ay<<','<<endl;
        ros::spinOnce();
        rate.sleep();
    }

    outfile.close();
    return 0;
}