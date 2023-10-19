#include <iostream>
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"
#include <nav_msgs/Odometry.h>
#include <chrono>
#include <ros/ros.h>
#include <pcl/io/obj_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include "Renderer.hpp"
#include <ekf/RadarMsg.h>
#define idEKF "idEKF"
#define idGT "idGT"
using namespace std;
using namespace lidar_obstacle_detection;

FusionEKF fusionEKF;
Renderer renderer;
Color colorGT  = Color(1,0,0); // red
Color colorEKF = Color(0,1,0); // green
int gtCount=0;
int ekfCount=0;
vector<VectorXd> estimations;
vector<VectorXd> ground_truth;

void writeResults(){

    // used to compute the RMSE later
    Tools tools;
    
    std::ofstream myfile;
    myfile.open("./res.txt", std::ios_base::app);
    for (size_t i = 0; i < estimations.size(); ++i) {
        VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth, (i+1));
	std::cout<< "estimations size : "<< estimations.size() <<std::endl;
        std::cout<< "gt size : "<< ground_truth.size() <<std::endl;
	std::cout<< "rmse : "<< RMSE <<std::endl;
        myfile << estimations[i][0] << " " << estimations[i][1] << " "
               << ground_truth[i][0] << " " << ground_truth[i][1] << " "
               << RMSE(0) << " " << RMSE(1) << std::endl;
    }
    myfile.close();
    
}


void lidarCb(const nav_msgs::Odometry::ConstPtr& msg){

    MeasurementPackage meas_package;
    meas_package.sensor_type_ = MeasurementPackage::LASER;
    meas_package.raw_measurements_ = VectorXd(2);

    meas_package.raw_measurements_ << msg->pose.pose.position.x, msg->pose.pose.position.y;
    meas_package.timestamp_ = msg->header.stamp.sec;
    fusionEKF.ProcessMeasurement(meas_package);    	  
    VectorXd estimate(4);
    estimate(0) = fusionEKF.ekf_.x_(0);
    estimate(1) = fusionEKF.ekf_.x_(1);
    estimate(2) = fusionEKF.ekf_.x_(2);
    estimate(3) = fusionEKF.ekf_.x_(3);
    estimations.push_back(estimate);
    renderer.addPointToViewer(fusionEKF.ekf_.x_(0), fusionEKF.ekf_.x_(1),"ekf",ekfCount,colorEKF);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << fusionEKF.ekf_.x_(0), fusionEKF.ekf_.x_(1), 0;
    // Apply the transformation
    renderer.updatePose(idEKF,transform);

    ekfCount++;
}

void gtCb(const nav_msgs::Odometry::ConstPtr& msg){
    renderer.addPointToViewer(msg->pose.pose.position.x, msg->pose.pose.position.y,"gt",gtCount,colorGT);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << fusionEKF.ekf_.x_(0), fusionEKF.ekf_.x_(1), 0;
    // Apply the transformation
    renderer.updatePose(idGT,transform);
    VectorXd gt_values(4);
    gt_values(0) = fusionEKF.ekf_.x_(0);
    gt_values(1) = fusionEKF.ekf_.x_(1); 
    gt_values(2) = 0;
    gt_values(3) = 0;
    ground_truth.push_back(gt_values);

    gtCount++;
}


void radarCb(const ekf::RadarMsg::ConstPtr& msg){

    MeasurementPackage meas_package;
    meas_package.sensor_type_ = MeasurementPackage::RADAR;

    meas_package.raw_measurements_ = VectorXd(3);
    
    meas_package.raw_measurements_ << msg->rho,msg->theta, msg->rho_dot;
    meas_package.timestamp_ = msg->timestamp;

    fusionEKF.ProcessMeasurement(meas_package);  
    VectorXd estimate(4);
    estimate(0) = fusionEKF.ekf_.x_(0);
    estimate(1) = fusionEKF.ekf_.x_(1);
    estimate(2) = fusionEKF.ekf_.x_(2);
    estimate(3) = fusionEKF.ekf_.x_(3);
    estimations.push_back(estimate);
    renderer.addPointToViewer(fusionEKF.ekf_.x_(0), fusionEKF.ekf_.x_(1),"ekf",ekfCount,colorEKF);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << fusionEKF.ekf_.x_(0), fusionEKF.ekf_.x_(1), 0;
    // Apply the transformation
    renderer.updatePose(idEKF,transform);
    ekfCount++;
}

int main(int argc,char **argv){
    std::ofstream myfile;

    myfile.open("./res.txt");
    myfile<<"estimate_x;estimate_y=;x_gt;y_gt;rmse_x=RMSE(0);rmse_y=RMSE(1)"<<std::endl;
    myfile.close();
    renderer.InitCamera(CameraAngle::TopDown);

    // Clear viewer
    renderer.ClearViewer();
    ros::init(argc, argv, "EKF");
    ros::NodeHandle n;
    std::cout<<"Start"<<std::endl;

    renderer.addCircle(0, 0, idEKF, 3.0,colorEKF);
    renderer.addCircle(0, 0, idGT, 3.0, colorGT);
    
    //Subscriber 
    ros::Subscriber lidar_sub          = n.subscribe<nav_msgs::Odometry>("/position_msg",10,&lidarCb); 
    ros::Subscriber radar_sub          = n.subscribe<ekf::RadarMsg>("/radar_msg",10,&radarCb); 
    ros::Subscriber gt_sub             = n.subscribe<nav_msgs::Odometry>("/gt_msg",10,&gtCb); 

    while(!renderer.WasViewerStopped()){
        ros::spinOnce();
        renderer.SpinViewerOnce();
	
    }
    writeResults();
    return 0;
}

