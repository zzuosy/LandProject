#include "ros/ros.h"
#include "std_msgs/String.h"
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/ObjectCount.h>
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
#include <LandProject/yolopose.h>
#include <LandProject/arucopose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
//话题回调函数
darknet_ros_msgs::ObjectCount object_count;
void ObjectCountcb(const darknet_ros_msgs::ObjectCount::ConstPtr& msg){
	object_count = *msg;
}

darknet_ros_msgs::BoundingBoxes tag_yolo;
//ROS_INFO_STREAM("WOQUQUUUU"<<tag_yolo);
void BoundingBoxescb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){
	tag_yolo = *msg;
    //ROS_INFO_STREAM("box: "<<tag_yolo);
}
cv::Mat im;
void imageCb(const sensor_msgs::ImageConstPtr& msg){

    // get time stamp for frame
    //stamp_ = ros::Time::now();
    //seq_ = stamp_.toSec();
    cv_bridge::CvImagePtr cv_ptr;

    try{
        //将收到的消息使用cv_bridge转移到全局变量图像指针cv_ptr中，其成员变量image就是Mat型的图片
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        im = cv_ptr->image;
        //ROS_INFO_STREAM("cols and rows"<<im.rows<<" "<<im.cols);
        //image_display_ = image_.clone();
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //char image_name[20];
    //sprintf(image_name, "%d.jpg", count);
    //count++;
    //string img_name = image_name;
    //cv::imwrite(img_name,image_);
    //处理图片信息
}

double solveyaw(int& iter, std::vector<darknet_ros_msgs::BoundingBox>& boxes, int& flag)
{
    //ROS_INFO("solve yaw");
    double yaw_estimate;
    if (iter == 3)
    {
        flag = 33;
        yaw_estimate = 0;
        /*
        //double width0 = boxes[0].xmax - boxes[0].xmin;
        //double width1 = boxes[1].xmax - boxes[1].xmin;
        double width2 = boxes[2].xmax - boxes[2].xmin;
        //ROS_INFO_STREAM("width "<<width0<<" "<<width1<<" "<<width2);
        double xcenter0 = (boxes[0].xmax + boxes[0].xmin)/2;
        double ycenter0 = (boxes[0].ymax + boxes[0].ymin)/2;
        double xcenter1 = (boxes[1].xmax + boxes[1].xmin)/2;
        double ycenter1 = (boxes[1].ymax + boxes[1].ymin)/2;
        double xcenter2 = (boxes[2].xmax + boxes[2].xmin)/2;
        double ycenter2 = (boxes[2].ymax + boxes[2].ymin)/2;
                
        //ROS_INFO_STREAM("width "<<xcenter0<<" "<<ycenter0<<" "<<xcenter1<<" "<<ycenter1<<" "<<xcenter2<<" "<<ycenter2);
        double rate0 = width0/width1;
        double rate1 = width1/width2;
        double rate2 = width0/width2;
        bool ensure = false;
        //ROS_INFO_STREAM("rate "<<rate0<<" "<<rate1<<" "<<rate2);
        if((0.3<rate0&&rate0<0.5)&&(0.55<rate1&&rate1<0.75)&&(0.15<rate2&&rate2<0.35)) ensure = true;
        //ROS_INFO_STREAM("ensure "<<ensure);
        if(ensure){
            flag = 3;
            if(ycenter2<ycenter0 && ycenter0<ycenter1){
                //ROS_INFO_STREAM("forward face");
                double yaw_estimate0 = atan((xcenter0-xcenter2)/(ycenter2-ycenter0));
                double yaw_estimate1 = atan((xcenter1-xcenter0)/(ycenter0-ycenter1));
                double yaw_estimate2 = atan((xcenter1-xcenter2)/(ycenter2-ycenter1));
                //ROS_INFO_STREAM("width "<<xcenter0<<" "<<ycenter0<<" "<<xcenter1<<" "<<ycenter1<<" "<<xcenter2<<" "<<ycenter2);
                //ROS_INFO_STREAM("yaw_estimate "<<yaw_estimate0<<" "<<yaw_estimate1<<" "<<yaw_estimate2);
                yaw_estimate = (yaw_estimate0+yaw_estimate1+yaw_estimate2)/3;
            }
            else if((ycenter2 == ycenter1)||(ycenter2 == ycenter0)||(ycenter1 == ycenter0))
            {
                //ROS_INFO_STREAM("zhongjian face");
                if(xcenter2 < xcenter1){
                    yaw_estimate = -0.5*3.14;
                }
                else{
                    yaw_estimate = 0.5*3.14;
                }
            }
            else{
                //ROS_INFO_STREAM("backward face");
                double yaw_estimate0 = atan((ycenter2-ycenter0)/(xcenter2-xcenter0));
                double yaw_estimate1 = atan((ycenter0-ycenter1)/(xcenter0-xcenter1));
                double yaw_estimate2 = atan((ycenter2-ycenter1)/(xcenter2-xcenter1));
                double temp = (yaw_estimate0+yaw_estimate1+yaw_estimate2)/3;
                yaw_estimate =  temp + temp/abs(temp)*90/180*3.14;
            }
        }
        else{
            yaw_estimate = 0;
        }*/
    }
    else if (iter == 4)
    {
        flag = 4;
        double xscenter = (boxes[0].xmax + boxes[0].xmin + boxes[1].xmax + boxes[1].xmin)/4;
        double yscenter = (boxes[0].ymax + boxes[0].ymin + boxes[1].ymax + boxes[1].ymin)/4;
        double xlcenter = (boxes[2].xmax + boxes[2].xmin + boxes[3].xmax + boxes[3].xmin)/4;
        double ylcenter = (boxes[2].ymax + boxes[2].ymin + boxes[3].ymax + boxes[3].ymin)/4;
        if(xlcenter<xscenter){
            yaw_estimate = atan((yscenter - ylcenter)/(xscenter - xlcenter));
        }
        else if (xlcenter == xscenter)
        {
            if(ylcenter>yscenter){
                yaw_estimate = -0.5*3.1415926;
            }
            else{
                yaw_estimate = 0.5*3.1415926;
            }
        }
        else{
            if(ylcenter == yscenter){
                yaw_estimate = -3.1415926;
            }
            else{
                double tmp = atan((xlcenter - xscenter)/(yscenter - ylcenter));
                yaw_estimate = tmp + tmp/abs(tmp)*0.5*3.1415926;
            }
        }
        
    }
    else if (iter == 5)
    {
        flag = 5;
        double xscenter = (boxes[1].xmax + boxes[1].xmin + boxes[2].xmax + boxes[2].xmin)/4;
        double yscenter = (boxes[1].ymax + boxes[1].ymin + boxes[2].ymax + boxes[2].ymin)/4;
        double xlcenter = (boxes[4].xmax + boxes[4].xmin + boxes[3].xmax + boxes[3].xmin)/4;
        double ylcenter = (boxes[4].ymax + boxes[4].ymin + boxes[3].ymax + boxes[3].ymin)/4;
        if(xlcenter<xscenter){
            yaw_estimate = atan((yscenter - ylcenter)/(xscenter - xlcenter));
        }
        else if (xlcenter == xscenter)
        {
            if(ylcenter>yscenter){
                yaw_estimate = -0.5*3.1415926;
            }
            else{
                yaw_estimate = 0.5*3.1415926;
            }
        }
        else{
            if(ylcenter == yscenter){
                yaw_estimate = -3.1415926;
            }
            else{
                double tmp = atan((xlcenter - xscenter)/(yscenter - ylcenter));
                yaw_estimate = tmp + tmp/abs(tmp)*0.5*3.1415926;
            }
        }
    }
    
    
    else{
        flag = 5;
        //ROS_INFO_STREAM("two much detected");
        yaw_estimate = 0;
    }

    yaw_estimate = yaw_estimate/3.1415926*180;
    if(yaw_estimate > 180){
        yaw_estimate -= 360;
    }
    if(yaw_estimate < -180){
        yaw_estimate += 360;
    }
    return yaw_estimate;
}

void solvepose(int& iter, std::vector<darknet_ros_msgs::BoundingBox>& boxes, std::vector<cv::Point2d>& image_points, std::vector<cv::Point3d>& image_points_in3d, double& yaw_estimate, cv::Mat& camMatrix, cv::Mat& distCoeff, cv::Mat& rvec, cv::Mat& tvec)
{
    double x1,y1,x2,y2,x3,y3,x4,y4;
    double xcenter0,ycenter0,xcenter1,ycenter1,xcenter2,ycenter2,xcenter3,ycenter3;
    if(iter == 4){
        xcenter0 = (boxes[0].xmax + boxes[0].xmin)/2;
        ycenter0 = (boxes[0].ymax + boxes[0].ymin)/2;
        xcenter1 = (boxes[1].xmax + boxes[1].xmin)/2;
        ycenter1 = (boxes[1].ymax + boxes[1].ymin)/2;
        xcenter2 = (boxes[2].xmax + boxes[2].xmin)/2;
        ycenter2 = (boxes[2].ymax + boxes[2].ymin)/2;
        xcenter3 = (boxes[3].xmax + boxes[3].xmin)/2;
        ycenter3 = (boxes[3].ymax + boxes[3].ymin)/2;
    }
    else{
        xcenter0 = (boxes[1].xmax + boxes[1].xmin)/2;
        ycenter0 = (boxes[1].ymax + boxes[1].ymin)/2;
        xcenter1 = (boxes[2].xmax + boxes[2].xmin)/2;
        ycenter1 = (boxes[2].ymax + boxes[2].ymin)/2;
        xcenter2 = (boxes[3].xmax + boxes[3].xmin)/2;
        ycenter2 = (boxes[3].ymax + boxes[3].ymin)/2;
        xcenter3 = (boxes[4].xmax + boxes[4].xmin)/2;
        ycenter3 = (boxes[4].ymax + boxes[4].ymin)/2;
    }


    if(-90<=yaw_estimate&&yaw_estimate<=90){
        if(ycenter2 > ycenter3){
            x1 = xcenter3;
            y1 = ycenter3;
            x2 = xcenter2;
            y2 = ycenter2;
        }
        else if (ycenter2 == ycenter3)
        {
            if(((xcenter2<xcenter3)&&yaw_estimate==-90) || ((xcenter2>xcenter3)&&yaw_estimate==90)){
                x1 = xcenter2;
                y1 = ycenter2;
                x2 = xcenter3;
                y2 = ycenter3;
            }
            else{
                x1 = xcenter3;
                y1 = ycenter3;
                x2 = xcenter2;
                y2 = ycenter2;
            }
        }
        else{
            x1 = xcenter2;
            y1 = ycenter2;
            x2 = xcenter3;
            y2 = ycenter3;
        }

        if(ycenter0 > ycenter1){
            x4 = xcenter1;
            y4 = ycenter1;
            x3 = xcenter0;
            y3 = ycenter0;
        }
        else if (ycenter0 == ycenter1)
        {
            if(((xcenter0<xcenter1)&&yaw_estimate==-90) || ((xcenter0>xcenter1)&&yaw_estimate==90)){
                x4 = xcenter0;
                y4 = ycenter0;
                x3 = xcenter1;
                y3 = ycenter1;
            }
            else{
                x4 = xcenter1;
                y4 = ycenter1;
                x3 = xcenter0;
                y3 = ycenter0;
            }
        }
        else{
            x4 = xcenter0;
            y4 = ycenter0;
            x3 = xcenter1;
            y3 = ycenter1;
        }
    }
    else{
        if(ycenter2 > ycenter3){
            x1 = xcenter2;
            y1 = ycenter2;
            x2 = xcenter3;
            y2 = ycenter3;
        }
        else{
            x1 = xcenter3;
            y1 = ycenter3;
            x2 = xcenter2;
            y2 = ycenter2;
        }

        if(ycenter0 > ycenter1){
            x4 = xcenter0;
            y4 = ycenter0;
            x3 = xcenter1;
            y3 = ycenter1;
        }
        else{
            x4 = xcenter1;
            y4 = ycenter1;
            x3 = xcenter0;
            y3 = ycenter0;
        }
    }
    image_points.push_back(cv::Point2d(x1,y1));
    image_points.push_back(cv::Point2d(x2,y2));
    image_points.push_back(cv::Point2d(x3,y3));
    image_points.push_back(cv::Point2d(x4,y4));
    //if(cv::solvePnP(image_points_in3d, image_points, camMatrix, distCoeff, rvec, tvec)){
    //    ROS_INFO_STREAM("RVEC: "<<rvec<<"TVEC: "<<tvec);
    //}
    cv::solvePnP(image_points_in3d, image_points, camMatrix, distCoeff, rvec, tvec);
    image_points.clear();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decopose_yolo");	//初始化ROS，节点命名为node_b，节点名必须唯一。
    ros::NodeHandle n;	//节点句柄实例化
    image_transport::ImageTransport it(n);

    ros::Rate rate(20.0);

    std::vector<int> tag_width;
    std::vector<int> tag_height;
    std::vector<int> tag_xcenter;
    std::vector<int> tag_ycenter;
    std::vector<cv::Point2d> image_points;
    std::vector<cv::Point3d> image_points_in3d;
    double x_center = 0;
    double y_center = 0;
    double yaw;

    cv::Mat rvec;
    cv::Mat tvec;
    int flag = 10;
    LandProject::yolopose pose;
    LandProject::arucopose arucopose;

    /*image_points_in3d.push_back(cv::Point3d(-0.0485,0.0485,0));
    image_points_in3d.push_back(cv::Point3d(-0.0485,-0.0485,0));
    image_points_in3d.push_back(cv::Point3d(0.0485,-0.0485,0));
    image_points_in3d.push_back(cv::Point3d(0.0485,0.0485,0));
    cv::Mat camMatrix = (cv::Mat_<double>(3, 3) << 563.113997, 0, 311.205711, 0, 563.859926, 211.569821, 0, 0, 1);
    cv::Mat distCoeff = (cv::Mat_<double>(1, 5) << -0.427374,0.195693,-0.000723,0.000705,0.00000);*/
    image_points_in3d.push_back(cv::Point3d(-0.2,0.2,0));
    image_points_in3d.push_back(cv::Point3d(-0.2,-0.2,0));
    image_points_in3d.push_back(cv::Point3d(0.2,-0.2,0));
    image_points_in3d.push_back(cv::Point3d(0.2,0.2,0));
    cv::Mat camMatrix = (cv::Mat_<double>(3, 3) << 592.981217466603, 0, 282.5038900011352, 0, 592.1763728186791, 212.2799732758863, 0, 0, 1);
    cv::Mat distCoeff = (cv::Mat_<double>(1, 5) << -0.4366727867512146,0.2057313607448225,0.001746426297464726,0.01872583732211049,0.00000);


    cv::Size camsize = cv::Size(640,480);
    //std::string cam_path;
    //nh_.getParam("cam_path",cam_path);
    aruco::CameraParameters camera(camMatrix, distCoeff,camsize);
    //ROS_INFO_STREAM("CAMPARA:"<<camera);
    //camera.readFromXMLFile(cam_path);
    aruco::MarkerDetector Detector;
    aruco::MarkerPoseTracker PoseTracker0;
    aruco::MarkerPoseTracker PoseTracker1;
    aruco::MarkerPoseTracker PoseTracker2;
    aruco::MarkerPoseTracker PoseTracker3;
    Detector.setDictionary("ARUCO_MIP_36h12");
    //cv::Mat im;
    //std::vector<aruco::Marker> markers;
    cv::Mat stran;
    cv::Mat rvecaruco;
    cv::Mat tvecaruco;
    cv::Mat rvecaruco0;
    cv::Mat tvecaruco0;
    cv::Mat rvecaruco1;
    cv::Mat tvecaruco1;
    cv::Mat rvecaruco2;
    cv::Mat tvecaruco2;
    cv::Mat rvecaruco3;
    cv::Mat tvecaruco3;
    double yawaruco;
    double yawaruco0;
    double yawaruco1;
    double yawaruco2;
    double yawaruco3;
    bool f1;
    bool f2;
    bool f3;
    bool f4;

    ros::Subscriber BoundingBoxes_sub = n.subscribe<darknet_ros_msgs::BoundingBoxes>
            ("/darknet_ros/bounding_boxes", 1, BoundingBoxescb);
    ros::Subscriber ObjectCount_sub = n.subscribe<darknet_ros_msgs::ObjectCount>
            ("/darknet_ros/found_object", 1, ObjectCountcb);
    ros::Publisher pose_pub = n.advertise<LandProject::yolopose>("/pose_yolo",1);
    ros::Publisher aruco_pub = n.advertise<LandProject::arucopose>("/pose_aruco",1);
    //ros::Subscriber image_sub = n.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 1, imageCb);
    image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw", 1, imageCb);

    while(ros::ok()){
        //ROS_INFO_STREAM("cols and rows"<<im.rows<<" "<<im.cols);
        arucopose.iffind = false;
        arucopose.pointstamped = geometry_msgs::PointStamped();
        if(im.empty()){
            ROS_INFO("image read failed");
        }
        else{
            f1 = false;
            f2 = false;
            f3 = false;
            f4 = false;
            auto markers = Detector.detect(im);
            for(auto m:markers){
                if(m.id == 0){
                    if(PoseTracker0.estimatePose(m, camera, 0.32)){
                        rvecaruco0 = PoseTracker0.getRvec();
                        tvecaruco0 = PoseTracker0.getTvec();
                        yawaruco0 = rvecaruco0.ptr<float>(0)[1];
                        m.draw(im);
                        //aruco::CvDrawingUtils::draw3dAxis(im,camera,rvecaruco0,tvecaruco0,0.1);
                        //geometry_msgs::Quaternion q;
                        //q=tf::createQuaternionMsgFromRollPitchYaw(rvecaruco.at<double>(0,0),rvecaruco.at<double>(2,0),rvecaruco.at<double>(1,0));
                        f1 = true;
                        //ROS_INFO("############");
                        //ROS_INFO_STREAM("ArUco time: "<<ros::Time::now());
                        //ROS_INFO_STREAM("ArUco0: "<<rvecaruco0<<"  "<<tvecaruco0);
                        //ROS_INFO("############");
                        //RTMatrix0 = PoseTracker.
                        //cv::imshow("image",im);
                        //cv::waitKey(1);
                    }
                }
                /*if(m.id == 1){
                    if(PoseTracker1.estimatePose(m, camera, 0.32)){
                        rvecaruco1 = PoseTracker1.getRvec();
                        tvecaruco1 = PoseTracker1.getTvec();
                        yawaruco1 = rvecaruco1.ptr<float>(0)[1];
                        m.draw(im);
                        //aruco::CvDrawingUtils::draw3dAxis(im,camera,rvecaruco1,tvecaruco1,0.1);
                        //geometry_msgs::Quaternion q;
                        //q=tf::createQuaternionMsgFromRollPitchYaw(rvecaruco.at<double>(0,0),rvecaruco.at<double>(2,0),rvecaruco.at<double>(1,0));
                        f2 = true;
                        //ROS_INFO("############");
                        //ROS_INFO_STREAM("ArUco time: "<<ros::Time::now());
                        //ROS_INFO_STREAM("ArUco1: "<<rvecaruco1<<"  "<<tvecaruco1);
                        //ROS_INFO("############");
                        //RTMatrix0 = PoseTracker.
                        //cv::imshow("image",im);
                        //cv::waitKey(1);
                    }
                }
                if(m.id == 2){
                    if(PoseTracker2.estimatePose(m, camera, 0.192)){
                        rvecaruco2 = PoseTracker2.getRvec();
                        tvecaruco2 = PoseTracker2.getTvec();
                        yawaruco2 = rvecaruco2.ptr<float>(0)[1];
                        m.draw(im);
                        //aruco::CvDrawingUtils::draw3dAxis(im,camera,rvecaruco2,tvecaruco2,0.1);
                        //geometry_msgs::Quaternion q;
                        //q=tf::createQuaternionMsgFromRollPitchYaw(rvecaruco.at<double>(0,0),rvecaruco.at<double>(2,0),rvecaruco.at<double>(1,0));
                        f3 = true;
                        //ROS_INFO("############");
                        //ROS_INFO_STREAM("ArUco time: "<<ros::Time::now());
                        //ROS_INFO_STREAM("ArUco2: "<<rvecaruco2<<"  "<<tvecaruco2);
                        //ROS_INFO("############");
                        //RTMatrix0 = PoseTracker.
                        //cv::imshow("image",im);
                        //cv::waitKey(1);
                    }
                }
                if(m.id == 3){
                    if(PoseTracker3.estimatePose(m, camera, 0.192)){
                        rvecaruco3 = PoseTracker3.getRvec();
                        tvecaruco3 = PoseTracker3.getTvec();
                        yawaruco3 = rvecaruco3.ptr<float>(0)[1];
                        m.draw(im);
                        //aruco::CvDrawingUtils::draw3dAxis(im,camera,rvecaruco3,tvecaruco3,0.1);
                        //geometry_msgs::Quaternion q;
                        //q=tf::createQuaternionMsgFromRollPitchYaw(rvecaruco.at<double>(0,0),rvecaruco.at<double>(2,0),rvecaruco.at<double>(1,0));
                        f4 = true;
                        //ROS_INFO("############");
                        //ROS_INFO_STREAM("ArUco time: "<<ros::Time::now());
                        //ROS_INFO_STREAM("ArUco3: "<<rvecaruco3<<"  "<<tvecaruco3);
                        //ROS_INFO("############");
                        //RTMatrix0 = PoseTracker.
                        //cv::imshow("image",im);
                        //cv::waitKey(1);
                    }
                }*/
                //ROS_INFO_STREAM("time: "<<"  "<<tvecaruco0<<"  "<<tvecaruco1<<"  "<<tvecaruco2<<"  "<<tvecaruco3);
            }
            //ROS_INFO_STREAM("time: "<<"  "<<tvecaruco0<<"  "<<tvecaruco1<<"  "<<tvecaruco2<<"  "<<tvecaruco3);
            /*if(f1&&f2&&f3&&f4){
                ROS_INFO_STREAM("ArUco: "<<tvecaruco<<"  "<<tvecaruco0<<"  "<<tvecaruco1<<"  "<<tvecaruco2<<"  "<<tvecaruco3);
                //ROS_INFO_STREAM("yaw: "<<yawaruco<<"  "<<yawaruco0<<"  "<<yawaruco1<<"  "<<yawaruco2<<"  "<<yawaruco3);
                tvecaruco = tvecaruco0 * 0.25 + tvecaruco1 * 0.25 + tvecaruco2 * 0.25 + tvecaruco3 * 0.25;
                yawaruco = yawaruco0 * 0.25 + yawaruco1 * 0.25 + yawaruco2 * 0.25 + yawaruco3 * 0.25;
                //ROS_INFO_STREAM(":::"<<tvecaruco<<"  "<<yawaruco);
                stran = tvecaruco;
                arucopose.iffind = true;
                arucopose.pointstamped.header.stamp = ros::Time::now();
                //ROS_INFO_STREAM("hahah"<<tvecaruco.type());
                ROS_INFO_STREAM("hahah"<<typeid(stran).name());
                arucopose.pointstamped.point.x = stran.ptr<float>(0)[0];
                arucopose.pointstamped.point.y = stran.ptr<float>(0)[1];
                arucopose.pointstamped.point.z = stran.ptr<float>(0)[2];
                //arucopose.pointstamped.point.x = 0;
                //arucopose.pointstamped.point.y = 1;
                //arucopose.pointstamped.point.z = 2;
                arucopose.yaw = yawaruco;
                f1 = false;
                f2 = false;
                f3 = false;
                f4 = false;
            }*/
            if(f1){
                ROS_INFO_STREAM("ArUco: "<<tvecaruco0);
                stran = tvecaruco0;
                arucopose.iffind = true;
                arucopose.pointstamped.header.stamp = ros::Time::now();
                arucopose.pointstamped.point.x = stran.ptr<float>(0)[0] + 0.2;
                arucopose.pointstamped.point.y = stran.ptr<float>(0)[1] + 0.2;
                arucopose.pointstamped.point.z = stran.ptr<float>(0)[2];
                f1 = false;
            }
            cv::imshow("image",im);
            cv::waitKey(1);
        }
        
        
        pose.iffind = false;
        pose.iflook = false;
        pose.posestamped = geometry_msgs::PoseStamped();
        x_center = 320;
        y_center = 240;
        pose.posestamped.pose.position.x = 0;
        pose.posestamped.pose.position.y = 0;
        pose.posestamped.pose.position.z = 0;

        if(object_count.count == 0){
            ROS_INFO_STREAM("no results");
            yaw = 0;
        }
        else{
            int tmpiters = tag_yolo.bounding_boxes.size();
            int count = 0;
            std::vector<darknet_ros_msgs::BoundingBox> filterboxes;
            for(int i = 0; i < tmpiters; i++){
                double nearrate = double(tag_yolo.bounding_boxes[i].xmax - tag_yolo.bounding_boxes[i].xmin)/double(tag_yolo.bounding_boxes[i].ymax - tag_yolo.bounding_boxes[i].ymin);
                //std::cout<<nearrate<<std::endl;
                if(nearrate < 0.7 || nearrate > 1.43){
                    count += 1;
                    continue;
                }
                filterboxes.push_back(tag_yolo.bounding_boxes[i]);
            }
            if(count == tmpiters){
                filterboxes.clear();
                filterboxes.assign(tag_yolo.bounding_boxes.begin(), tag_yolo.bounding_boxes.end());
            }
            int iters = filterboxes.size();
            //ros::Time yolo_time = tag_yolo.header.stamp;
            //ROS_INFO_STREAM("tmpiters"<<tmpiters);
            //ROS_INFO_STREAM("count"<<count);
            //ROS_INFO_STREAM("iters"<<iters);
            sort(filterboxes.begin(), filterboxes.end(), [](darknet_ros_msgs::BoundingBox a, darknet_ros_msgs::BoundingBox b){return (a.xmax-a.xmin) < (b.xmax-b.xmin);});

            
            yaw = 0;
            

            if(0<iters && iters < 4)
            {
                /*if(iters == 1){
                    x_center = (filterboxes[0].xmax + filterboxes[0].xmin)/2;
                    y_center = (filterboxes[0].ymax + filterboxes[0].ymin)/2;
                }
                else{
                    double rateee =  double(filterboxes[0].xmax - filterboxes[0].xmin)/double(filterboxes[1].xmax - filterboxes[1].xmin);
                    if(rateee<0.55){
                        x_center = (tag_yolo.bounding_boxes[0].xmax + tag_yolo.bounding_boxes[0].xmin)/2;
                        y_center = (tag_yolo.bounding_boxes[0].ymax + tag_yolo.bounding_boxes[0].ymin)/2;
                    }
                    else{
                        for(int i = 0; i < iters; i++){
                            x_center = x_center + (tag_yolo.bounding_boxes[i].xmax + tag_yolo.bounding_boxes[i].xmin)/(2*iters);
                            y_center = y_center + (tag_yolo.bounding_boxes[i].ymax + tag_yolo.bounding_boxes[i].ymin)/(2*iters);
                        }
                    }
                }*/
                x_center = (filterboxes[0].xmax + filterboxes[0].xmin)/2;
                y_center = (filterboxes[0].ymax + filterboxes[0].ymin)/2;
                pose.iflook = true;
                pose.posestamped.header.stamp = tag_yolo.header.stamp;
                pose.xcenter = x_center;
                pose.ycenter = y_center;
                ROS_INFO_STREAM("3  "<<x_center<<" "<<y_center);
                x_center = 0;
                y_center = 0;

            }
            if(iters == 4){
                double rateeee =  double(filterboxes[0].xmax - filterboxes[0].xmin)/double(filterboxes[1].xmax - filterboxes[1].xmin);
                double rateeee1 =  double(filterboxes[1].xmax - filterboxes[1].xmin)/double(filterboxes[2].xmax - filterboxes[2].xmin);
                double rateeee2 =  double(filterboxes[2].xmax - filterboxes[2].xmin)/double(filterboxes[3].xmax - filterboxes[3].xmin);
                ROS_INFO_STREAM("rate  "<<rateeee<<" "<<rateeee1<<" "<<rateeee2);
                if(rateeee<0.8){
                    pose.iflook = true;
                    pose.posestamped.header.stamp = tag_yolo.header.stamp;
                    x_center = (filterboxes[0].xmax + filterboxes[0].xmin)/2;
                    y_center = (filterboxes[0].ymax + filterboxes[0].ymin)/2;
                    pose.xcenter = x_center;
                    pose.ycenter = y_center;
                    ROS_INFO_STREAM("4  "<<x_center<<" "<<y_center);
                }
                else{
                    yaw = solveyaw(iters, filterboxes, flag);
                    solvepose(iters, filterboxes, image_points, image_points_in3d, yaw, camMatrix, distCoeff, rvec, tvec);
                    ROS_INFO_STREAM("4  "<<tvec);
                    pose.iffind = true;
                    pose.yaw = yaw/180*3.1415926;
                    pose.posestamped.header.stamp = tag_yolo.header.stamp;
                    pose.posestamped.pose.position.x = tvec.at<double>(0,0);
                    pose.posestamped.pose.position.y = tvec.at<double>(1,0);
                    pose.posestamped.pose.position.z = tvec.at<double>(2,0);
                    geometry_msgs::Quaternion quaternion;
                    quaternion=tf::createQuaternionMsgFromRollPitchYaw(rvec.at<double>(0,0),rvec.at<double>(2,0),rvec.at<double>(1,0));
                    pose.posestamped.pose.orientation = quaternion;
                }
            }
            if(iters > 4){
                yaw = solveyaw(iters, filterboxes, flag);
                solvepose(iters, filterboxes, image_points, image_points_in3d, yaw, camMatrix, distCoeff, rvec, tvec);
                ROS_INFO_STREAM("5  "<<tvec);
                pose.iffind = true;
                pose.yaw = yaw/180*3.1415926;
                pose.posestamped.header.stamp = tag_yolo.header.stamp;
                pose.posestamped.pose.position.x = tvec.at<double>(0,0);
                pose.posestamped.pose.position.y = tvec.at<double>(1,0);
                pose.posestamped.pose.position.z = tvec.at<double>(2,0);
                geometry_msgs::Quaternion quaternion;
                quaternion=tf::createQuaternionMsgFromRollPitchYaw(rvec.at<double>(0,0),rvec.at<double>(2,0),rvec.at<double>(1,0));
                pose.posestamped.pose.orientation = quaternion;
                
            }

            ROS_INFO_STREAM("YAW_estimate:  "<<yaw);
            //ROS_INFO_STREAM("flag"<<flag);
            
            flag = 10;
            //ROS_INFO_STREAM("A&B: "<<a<<" "<<b);
        }
        aruco_pub.publish(arucopose);
        pose_pub.publish(pose);
        //double a = tag_yolo.bounding_boxes[0].xmax;
        //double b = tag_yolo.bounding_boxes[0].ymax;
        //double a = 2;
        //double b = 3;
        //ROS_INFO_STREAM("A&B: "<<a<<" "<<b);
        //ROS_INFO_STREAM("what: "<<tag_yolo.header);
        ros::spinOnce();
        rate.sleep();
    }
    
	//ros::spin();	//程序进入循环，直到ros::ok()返回false，进程结束。
 
	return 0;
}
