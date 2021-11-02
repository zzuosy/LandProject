#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <armadillo>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cstdlib>
#include <cmath>

using namespace std;
using namespace arma;


sensor_msgs::Imu imudata;
geometry_msgs::Vector3 lin_accel;
void uavaccelcb(const sensor_msgs::Imu::ConstPtr& msg){
	imudata = *msg;
	lin_accel = imudata.linear_acceleration;
}

sensor_msgs::Imu carimudata;
geometry_msgs::Vector3 carlin_accel;
void caraccelcb(const sensor_msgs::Imu::ConstPtr& msg){
	carimudata = *msg;
	carlin_accel = carimudata.linear_acceleration;
}

std::string car_name("turtlebot3_waffle_pi");
std::string uav_name("iris");
gazebo_msgs::ModelStates models_state;
geometry_msgs::Pose car_pose;
geometry_msgs::Pose uav_pose;
void uwbcb(const gazebo_msgs::ModelStates::ConstPtr& msg){
	models_state = *msg;
    std::vector<string>::iterator iter1;
	iter1 = find(models_state.name.begin(), models_state.name.end(), car_name);
    std::vector<string>::iterator iter2;
	iter2 = find(models_state.name.begin(), models_state.name.end(), uav_name);
	auto index1 = distance(models_state.name.begin(), iter1);
	auto index2 = distance(models_state.name.begin(), iter2);
	car_pose = models_state.pose[index1];
	uav_pose = models_state.pose[index2];
}

class kalmanfilter{
private:
	Eigen::MatrixXd A;
	Eigen::MatrixXd B;
	Eigen::MatrixXd Q;
	Eigen::MatrixXd R;

	Eigen::VectorXd x_;
	Eigen::VectorXd x_m;
	Eigen::MatrixXd p_;
	Eigen::MatrixXd p_m;
public:
    kalmanfilter();
	~kalmanfilter();
    void init(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd q, Eigen::MatrixXd r);
	void calculate(Eigen::VectorXd u, Eigen::VectorXd z);
	Eigen::VectorXd getstate();
};

void kalmanfilter::init(Eigen::MatrixXd a, Eigen::MatrixXd b, Eigen::MatrixXd q, Eigen::MatrixXd r){
	A = a;
	B = b;
	Q = q;
	R = r;

	int n_states = A.cols();

    //x = [x, y, z, vx, vy, vz, bx, by, bz] personal version
	//x = [x, vx, bx, y, vy, by, z, vz, bz] now and referrence version
	//u = [ax, ay, az]
	//z = [x, y, z]
	x_.resize(n_states);
	x_m.resize(n_states);
	x_m << -1, 0, -0.25, 1, 0, 0.25, 2, 0, 0;
	p_.resize(n_states,n_states);
	p_m.resize(n_states,n_states);
	p_m.setIdentity();
}

kalmanfilter::kalmanfilter(){

}

kalmanfilter::~kalmanfilter(){

}

void kalmanfilter::calculate(Eigen::VectorXd u, Eigen::VectorXd z){

    Eigen::MatrixXd H(4,9);
    //double r_pred1 = std::sqrt( std::pow(x_m(0) - 2, 2) + std::pow(x_m(3) - 2, 2) + std::pow(x_m(6), 2) );
    //double r_pred2 = std::sqrt( std::pow(x_m(0) - 2, 2) + std::pow(x_m(3) + 2, 2) + std::pow(x_m(6), 2) );
    //double r_pred3 = std::sqrt( std::pow(x_m(0) + 2, 2) + std::pow(x_m(3) + 2, 2) + std::pow(x_m(6), 2) );
    //double r_pred4 = std::sqrt( std::pow(x_m(0) + 2, 2) + std::pow(x_m(3) - 2, 2) + std::pow(x_m(6), 2) );
	double r_pred1 = std::sqrt( std::pow(x_m(0) - 0.4, 2) + std::pow(x_m(3) - 0.4, 2) + std::pow(x_m(6), 2) );
    double r_pred2 = std::sqrt( std::pow(x_m(0) - 0.4, 2) + std::pow(x_m(3) + 0.4, 2) + std::pow(x_m(6), 2) );
    double r_pred3 = std::sqrt( std::pow(x_m(0) + 0.4, 2) + std::pow(x_m(3) + 0.4, 2) + std::pow(x_m(6), 2) );
    double r_pred4 = std::sqrt( std::pow(x_m(0) + 0.4, 2) + std::pow(x_m(3) - 0.4, 2) + std::pow(x_m(6), 2) );

	Eigen::VectorXd r_pred(4);
	r_pred << r_pred1, r_pred2, r_pred3, r_pred4;

    //H << (x_m(0) - 2)/r_pred1, 0, 0, (x_m(3) - 2)/r_pred1, 0, 0, (x_m(6) - 0)/r_pred1, 0, 0,
    //     (x_m(0) - 2)/r_pred2, 0, 0, (x_m(3) + 2)/r_pred2, 0, 0, (x_m(6) - 0)/r_pred2, 0, 0,
    //     (x_m(0) + 2)/r_pred3, 0, 0, (x_m(3) + 2)/r_pred3, 0, 0, (x_m(6) - 0)/r_pred3, 0, 0,
    //     (x_m(0) + 2)/r_pred4, 0, 0, (x_m(3) - 2)/r_pred4, 0, 0, (x_m(6) - 0)/r_pred4, 0, 0;


    H << (x_m(0) - 0.4)/r_pred1, 0, 0, (x_m(3) - 0.4)/r_pred1, 0, 0, (x_m(6) - 0)/r_pred1, 0, 0,
         (x_m(0) - 0.4)/r_pred2, 0, 0, (x_m(3) + 0.4)/r_pred2, 0, 0, (x_m(6) - 0)/r_pred2, 0, 0,
         (x_m(0) + 0.4)/r_pred3, 0, 0, (x_m(3) + 0.4)/r_pred3, 0, 0, (x_m(6) - 0)/r_pred3, 0, 0,
         (x_m(0) + 0.4)/r_pred4, 0, 0, (x_m(3) - 0.4)/r_pred4, 0, 0, (x_m(6) - 0)/r_pred4, 0, 0;

	x_ = A * x_m + B * u;
	p_ = A * p_m * A.transpose()  + Q;

	//cout << x_ << endl;
	//cout << H << endl;

	//cout<<"process value: "<<x_<<" "<<p_;
	//Eigen::MatrixXd tmp = H * p_ * H.transpose()  + R;
	//cout<<tmp;

	Eigen::MatrixXd k = p_ * H.transpose() *  (H * p_ * H.transpose()  + R).inverse();
	x_m = x_ + k * (z - r_pred);
	p_m = p_ - k * H * p_;

	//cout << k <<endl;
	//cout << x_m << endl;
}

Eigen::VectorXd kalmanfilter::getstate(){
	return x_m;
}

double gaussrand()
{
    static double V1, V2, S;
    static int phase = 0;
    double X;
     
    if ( phase == 0 ) {
        do {
            double U1 = (double)rand() / RAND_MAX;
            double U2 = (double)rand() / RAND_MAX;
             
            V1 = 2 * U1 - 1;
            V2 = 2 * U2 - 1;
            S = V1 * V1 + V2 * V2;
        } while(S >= 1 || S == 0);
         
        X = V1 * sqrt(-2 * log(S) / S);
    } else
        X = V2 * sqrt(-2 * log(S) / S);
         
    phase = 1 - phase;

	X = X * 0.01;
 
    return X;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "FiterNavigation");
	ros::NodeHandle n;

	ros::Rate rate(10);

    ros::Subscriber uavaccel_sub = n.subscribe<sensor_msgs::Imu>("/mavros/imu/data",1,uavaccelcb);
	ros::Subscriber caraccel_sub = n.subscribe<sensor_msgs::Imu>("/imu/",1,caraccelcb);
    ros::Subscriber uwb_sub = n.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 1, uwbcb);

	double dT = 0.1;

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9,9);
	Eigen::MatrixXd block_A(3,3);
    block_A << 1, dT, -dT*dT/2.0,
               0, 1 ,        -dT,
               0, 0 ,          1;
    A.block<3,3>(0,0) = block_A;
    A.block<3,3>(3,3) = block_A;
    A.block<3,3>(6,6) = block_A;

	/*A << 1, 0, 0, dT, 0 , 0 , -dT*dT/2.0, 0         , 0         ,
         0, 1, 0, 0 , dT, 0 , 0         , -dT*dT/2.0, 0         ,
		 0, 0, 1, 0 , 0 , dT, 0         , 0         , -dT*dT/2.0,
		 0, 0, 0, 1 , 0 , 0 , -dT       , 0         , 0         ,
		 0, 0, 0, 0 , 1 , 0 , 0         , -dT       , 0         ,
		 0, 0, 0, 0 , 0 , 1 , 0         , 0         , -dT       ,
		 0, 0, 0, 0 , 0 , 0 , 1         , 0         , 0         ,
		 0, 0, 0, 0 , 0 , 0 , 0         , 1         , 0         ,
		 0, 0, 0, 0 , 0 , 0 , 0         , 0         , 1         ;*/
	
	
	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(9,3);
	Eigen::MatrixXd block_B(3,3);
    block_B << dT*dT/2.0,  0,  0,
               dT       ,  0,  0,
               0        ,  0,  0;
    B.block<3,3>(0,0) = block_B;
    B.block<3,3>(3,3) = block_B;
    B.block<3,3>(6,6) = block_B;

	/*B << dT*dT/2.0, 0        , 0        ,
         0        , dT*dT/2.0, 0        ,
		 0        , 0        , dT*dT/2.0,
		 dT       , 0        , 0        ,
		 0        , dT       , 0        ,
		 0        , 0        , dT       ,
		 0        , 0        , 0        ,
		 0        , 0        , 0        ,
		 0        , 0        , 0        ;*/
	
	
	double m_tao_acc_sqrt = 0.3;
	double m_tao_bias_sqrt = 0.001;
	double tao_acc = m_tao_acc_sqrt * m_tao_acc_sqrt;
    double tao_bias = m_tao_bias_sqrt * m_tao_bias_sqrt;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(9,9);
    Eigen::MatrixXd block_Q(3,3);
    block_Q <<
         std::pow(dT,3)/3.0*tao_acc+std::pow(dT,5)/20.0*tao_bias,  std::pow(dT,2)/2*tao_acc+std::pow(dT,4)/8.0*tao_bias,  -std::pow(dT,3)/6*tao_bias,
         std::pow(dT,2)/2.0*tao_acc+std::pow(dT,4)/8.0*tao_bias ,  dT*tao_acc+std::pow(dT,3)/3*tao_bias                ,  -std::pow(dT,2)/2*tao_bias,
         -std::pow(dT,3)/6.0*tao_bias                           ,  -std::pow(dT,2)/2*tao_bias                          ,  dT*tao_bias               ;
    Q.block<3,3>(0,0) = block_Q;
    Q.block<3,3>(3,3) = block_Q;
    Q.block<3,3>(6,6) = block_Q;

	Eigen::Matrix4d R;
	R << 0.1, 0  , 0  , 0  ,
	     0  , 0.1, 0  , 0  ,
		 0  , 0  , 0.1, 0  ,
         0  , 0  , 0  , 0.1;
    



	//Eigen::VectorXd w;
	//w << 0.1, 0.2, 0.1, 0.3, 0.1, 0.2, 0.1, 0.1, 0.1;
	//Eigen::MatrixXd Q = w * w.transpose() ;

	//mat Q = { {0.01, 0.02, 0.01, 0.03, 0.01, 0.02},
    //          {0.02, 0.04, 0.02, 0.06, 0.02, 0.04},
	//		  {0.01, 0.02, 0.01, 0.03, 0.01, 0.02},
	//		  {0.03, 0.06, 0.03, 0.09, 0.03, 0.06},
	//		  {0.01, 0.02, 0.01, 0.03, 0.01, 0.02},
	//		  {0.02, 0.04, 0.02, 0.06, 0.02, 0.04} };
	
	//Eigen::VectorXd v;
	//v << 0.1, 0.2, 0.1;
	//Eigen::MatrixXd R = v * v.transpose() ;

	//mat R = { {0.01, 0.02, 0.01},
    //          {0.02, 0.04, 0.02},
	//		  {0.01, 0.02, 0.01} };
	
	kalmanfilter kf;
	kf.init(A, B, Q, R);

	while(ros::ok()){
		double dx = uav_pose.position.x - car_pose.position.x;
		double dy = uav_pose.position.y - car_pose.position.y;
		double dz = uav_pose.position.z - car_pose.position.z;        
		double dx1 = uav_pose.position.x - car_pose.position.x - 0.4;
		double dy1 = uav_pose.position.y - car_pose.position.y - 0.4;
		double dz1 = uav_pose.position.z - car_pose.position.z;
		double dx2 = uav_pose.position.x - car_pose.position.x - 0.4;
		double dy2 = uav_pose.position.y - car_pose.position.y + 0.4;
		double dz2 = uav_pose.position.z - car_pose.position.z;
        double dx3 = uav_pose.position.x - car_pose.position.x + 0.4;
		double dy3 = uav_pose.position.y - car_pose.position.y + 0.4;
		double dz3 = uav_pose.position.z - car_pose.position.z;
        double dx4 = uav_pose.position.x - car_pose.position.x + 0.4;
		double dy4 = uav_pose.position.y - car_pose.position.y - 0.4;
		double dz4 = uav_pose.position.z - car_pose.position.z;
		/*double dx1 = uav_pose.position.x - 2;
		double dy1 = uav_pose.position.y - 2;
		double dz1 = uav_pose.position.z - 0;
		double dx2 = uav_pose.position.x - 2;
		double dy2 = uav_pose.position.y + 2;
		double dz2 = uav_pose.position.z - 0;
        double dx3 = uav_pose.position.x + 2;
		double dy3 = uav_pose.position.y + 2;
		double dz3 = uav_pose.position.z - 0;
        double dx4 = uav_pose.position.x + 2;
		double dy4 = uav_pose.position.y - 2;
		double dz4 = uav_pose.position.z - 0;*/
        double z1 = std::sqrt(std::pow(dx1, 2) + std::pow(dy1, 2) + std::pow(dz1, 2)) +  gaussrand();
        double z2 = std::sqrt(std::pow(dx2, 2) + std::pow(dy2, 2) + std::pow(dz2, 2)) +  gaussrand();
        double z3 = std::sqrt(std::pow(dx3, 2) + std::pow(dy3, 2) + std::pow(dz3, 2)) +  gaussrand();
        double z4 = std::sqrt(std::pow(dx4, 2) + std::pow(dy4, 2) + std::pow(dz4, 2)) +  gaussrand();
		//dx = dx + gaussrand();
		//dy = dy + gaussrand();
		//dz = dz + gaussrand();
		double ax = lin_accel.x - carlin_accel.x;
		double ay = lin_accel.y - carlin_accel.y;
		double az = lin_accel.z - carlin_accel.z;

		ROS_INFO_STREAM("IMU: "<<lin_accel.x<<" "<<lin_accel.y<<" "<<lin_accel.z<<" "<<"DISTANCE: "<<dx<<" "<<dy<<" "<<dz);
		ROS_INFO_STREAM("D "<<dx1<<" "<<dy1<<" "<<dz1<<" "<<dx2<<" "<<dy2<<" "<<dz2<<" "<<dx3<<" "<<dy3<<" "<<dz3<<" "<<dx4<<" "<<dy4<<" "<<dz4);
		ROS_INFO_STREAM("Z"<<z1<<" "<<z2<<" "<<z3<<" "<<z4);
		if(dx != 0){
			Eigen::VectorXd u(9);
			u << ax, 0, 0, ay, 0, 0, az, 0, 0;
			Eigen::VectorXd z(4);
			z << z1, z2, z3, z4;
            //double z = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2)) +  gaussrand();
			kf.calculate(u, z);
			Eigen::VectorXd x_state = kf.getstate();
			ROS_INFO_STREAM("kalman: "<<x_state);
		}
		ros::spinOnce();
		rate.sleep();
	}
}