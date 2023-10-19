#include "kf.h"

KalmanFilter::KalmanFilter() {
}

KalmanFilter::~KalmanFilter() {
}

void KalmanFilter::Predict() {
  /**
	TODO: 
	* predict the state
	*/
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

  /**
  TODO: 
  * update the state by using Kalman Filter equations
  */
  VectorXd y =  z - H_ * x_;
  MatrixXd Ht = H_.transpose();//use the transpose() function
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse(); //use the inverse() function
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  /**
  TODO: 
  * Compute the new estimate
  */
  x_ = x_ + K * y;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
