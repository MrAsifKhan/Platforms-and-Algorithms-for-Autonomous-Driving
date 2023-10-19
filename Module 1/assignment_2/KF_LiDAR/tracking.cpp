#include "tracking.h"
#include <iostream>
#include "Eigen/Dense"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

Tracking::Tracking() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  // create a 4D state vector, we don't know yet the values of the x state
  kf_.x_ = VectorXd(4);

  /*************TODO*************/
  // state covariance matrix P
  kf_.P_ = MatrixXd(4, 4);
  kf_.P_ << 10, 0, 0, 0,
            0, 10, 0, 0,
            0, 0, 10, 0,
            0, 0, 0, 10;


  // measurement covariance
  kf_.R_ = MatrixXd(2, 2);
  kf_.R_ << 0.0225, 0,
            0, 0.0225;

  // measurement matrix
  kf_.H_ = MatrixXd(2, 4);
  kf_.H_ << 1, 0, 0, 0,
            0, 1, 0, 0;

  /*************TODO*************/
  // the initial transition matrix F_
  kf_.F_ = MatrixXd(4, 4);
  kf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  // set the acceleration noise components
  noise_ax = 5;
  noise_ay = 5;
}

Tracking::~Tracking() {

}


// Process a single measurement
void Tracking::ProcessMeasurement(const MeasurementPackage &measurement_pack,vector<float> &x_v,vector<float> &y_v) {
  if (!is_initialized_) {
    //cout << "Kalman Filter Initialization " << endl;

    // set the state with the initial location and zero velocity
    kf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;

    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  
  /*************TODO*************/
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  kf_.F_(0, 2) = dt;
  kf_.F_(1, 3) = dt;

  /*************TODO*************/
  // set the process covariance matrix Q
  kf_.Q_ = MatrixXd(4, 4);
  kf_.Q_ <<  dt_4/4 * noise_ax, 0, dt_3/2 * noise_ax, 0,
		         0, dt_4/4 * noise_ay, 0, dt_3/2 * noise_ay,
			       dt_3/2 * noise_ax, 0, dt_2 * noise_ax, 0,
			       0, dt_3 * noise_ay, 0, dt_2 * noise_ay;

  /*************TODO*************/
  // call the predict and update functions
  //REMIND that update uses data measurement_pack.raw_measurements_!!
  //predict
  kf_.Predict();
  //update
  kf_.Update(measurement_pack.raw_measurements_);
  x_v.push_back(kf_.x_[0]);
  y_v.push_back(kf_.x_[1]);
  //cout << "P_= " << kf_.P_ << endl;
}
