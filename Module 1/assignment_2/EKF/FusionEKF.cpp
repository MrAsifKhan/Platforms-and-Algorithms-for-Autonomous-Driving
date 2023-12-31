#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
	is_initialized_ = false;

	previous_timestamp_ = 0;

	// initializing matrices
	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

	//measurement covariance matrix - laser
	R_laser_ << 0.0225, 0,
		0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ << 0.09, 0, 0,
				0, 0.0009, 0,
				0, 0, 0.09;
		
	//measurement matrix
	H_laser_ = MatrixXd(2, 4);
	H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

	/**
	* Finish initializing the FusionEKF.
	* Set the process and measurement noises
	*/
	//create a 4D state vector, we don't know yet the values of the x state
	ekf_.x_ = VectorXd(4);

	/** TODO
	 * Initialize the state matrix P
	*/
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;

	//the initial transition matrix F_
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;
	noise_ax = 9;
	noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


	/*****************************************************************************
	*  Initialization
	****************************************************************************/
        std::cout<<"inside 0.5555"<<std::endl;
	std::cout<<"is_intilaized:" << is_initialized_ <<std::endl;
	if (!is_initialized_) {
		/**
		  * Initialize the state ekf_.x_ with the first measurement.
		  * Create the covariance matrix.
		  * Remember: you'll need to convert radar from polar to cartesian coordinates.
		*/
		// first measurement
		std::cout<<"inside 0.9999"<<std::endl;
		cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4);
		std::cout<<"inside 1"<<std::endl;
		previous_timestamp_ = measurement_pack.timestamp_;
		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/

			float rho = measurement_pack.raw_measurements_[0];
			float phi = measurement_pack.raw_measurements_[1];
			float rho_dot = measurement_pack.raw_measurements_[2];

			std::cout << "1:: " << rho << std::endl;
			std::cout << "2:: " << rho_dot << std::endl;
                        std::cout << "3:: " << phi << std::endl;
                        std::cout<<"start of radarcb 22"<<std::endl;
			std::cout << "1:: " << rho << std::endl;
			std::cout << "2:: " << rho_dot << std::endl;
                        std::cout << "3:: " << phi << std::endl;
			std::cout << "4:: " << rho_dot*sin(phi) << std::endl;
			ekf_.x_ << rho*cos(phi),rho*sin(phi),rho_dot*cos(phi),rho_dot*sin(phi);
                        std::cout<<"start of radarcb 22.555"<<std::endl;
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
			/**
			Initialize state.
			*/
			ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
		}
		std::cout<<"end of inside"<<std::endl;
 
		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/*****************************************************************************
	*  Prediction
	****************************************************************************/
	/**
	 * Update the state transition matrix F according to the new elapsed time.
	  - Time is measured in seconds.
	*/
	//compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;


	/** TODO
	 * Modify the F matrix so that the time is integrated
	*/
	
	float dt_2 = dt*dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;
	ekf_.F_ << 1, 0, dt, 0,
			  0, 1, 0, dt,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

	/** TODO
	 * set the process covariance matrix Q
	 * use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	*/
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
           0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
           dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
           0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;
	ekf_.Predict();

	/*****************************************************************************
	*  Update
	****************************************************************************/

	/**
	 * Use the sensor type to perform the update step.
	 * Update the state and covariance matrices.
	*/

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
		Tools t_;
		/** TODO: 
	 	* Calculate the jacobian and call the UpdateEKF function
		*/
		ekf_.H_=t_.CalculateJacobian(ekf_.x_);
		ekf_.R_=R_radar_;
		ekf_.UpdateEKF(measurement_pack.raw_measurements_);
		//call the updateEKF function
	} else {
		ekf_.H_=H_laser_;
		ekf_.R_=R_laser_;
		/** TODO: 
	 	* Calculate the update function
		*/
		//call the update function
		ekf_.Update(measurement_pack.raw_measurements_);
	}

	// print the  output
	cout << "x_ = " << ekf_.x_ << endl;
	cout << "P_ = " << ekf_.P_ << endl;
}
