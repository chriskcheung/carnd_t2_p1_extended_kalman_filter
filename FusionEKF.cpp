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
              0,      0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0,      0,
              0,    0.0009, 0,
              0,    0,      0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  Hj_ << 1., 1., 0,  0,
         1., 1., 0,  0,
         1., 1., 1., 1.;
		
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    VectorXd x = VectorXd(4);
    //x << 1, 1, 1, 1;
    MatrixXd P = MatrixXd(4,4);
    P << 1, 0, 0, 0,
	     0, 1, 0, 0,
	     0, 0, 1000, 0,
		 0, 0, 0, 1000;
	MatrixXd F = MatrixXd(4,4);
    F << 1, 0, 0., 0,
	     0, 1, 0, 0.,
	     0, 0, 1, 0,
		 0, 0, 0, 1;
    MatrixXd Q = MatrixXd(4,4);
    Q << 0., 0, 0., 0,
	     0, 0., 0, 0.,
	     0., 0, 0., 0,
		 0, 0., 0, 0.;
    cout << "fusionekf(72): ############## INITIALIZE STATE X ###########" << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      For a row containing radar data, the columns are: 
	    sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.
      */
	  float rho    = measurement_pack.raw_measurements_(0); // rho_measured
	  float phi    = measurement_pack.raw_measurements_(1); // phi_measured
	  float rhodot = measurement_pack.raw_measurements_(2); // rhodot_measured
	  cout << "fusionekf(85): rho|phi|rhodot=" << rho <<"|"<< phi <<"|"<< rhodot << endl;
	  float px     = cos(phi)*rho;	   // calculate position of x from radian metric phi and rho
	  float py     = sin(phi)*rho;     // calculate position of y from radian metric phi and rho
	  float vx     = cos(phi)*rhodot;  // calculate velocity of x from radian metric phi and rhodot
	  float vy     = sin(phi)*rhodot;  // calculate velocity of y from radian metric phi and rhodot
	  
	  x << px, py, 0, 0;  // use 0 for vx and vy for intitialization due to radar measurement doesn't have enough info to determine vx and vy
	  ekf_.Init(x, P, F, Hj_, R_radar_, Q);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      For a row containing lidar data, the columns are: 
	    sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.
      */
	  float px     = measurement_pack.raw_measurements_(0); // x_measured
	  float py     = measurement_pack.raw_measurements_(1); // y_measured
	
	  x << px, py, 0, 0;  // for vx and vy, initialize to 0 for now
	  ekf_.Init(x, P, F, H_laser_, R_radar_, Q);
    }
	cout << "FustionEKV(117) ekf_.x_="   << endl << ekf_.x_ << endl;
	cout << "FustionEKV(117) ekf_.P_=\n" << endl << ekf_.P_ << endl;
	cout << "FustionEKV(117) ekf_.F_=\n" << endl << ekf_.F_ << endl;
	cout << "FustionEKV(117) ekf_.H_=\n" << endl << ekf_.H_ << endl;
	cout << "FustionEKV(117) ekf_.R_=\n" << endl << ekf_.R_ << endl;
	cout << "FustionEKV(117) ekf_.Q_=\n" << endl << ekf_.Q_ << endl;

	// update previous_timestamp with current measurement timestamp
    previous_timestamp_ = measurement_pack.timestamp_; // timestamp_measured in microsecond unit;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

   // calculate delta timestamp
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
   
   // update F matrix
   ekf_.F_ << 1, 0, dt, 0, 
              0, 1, 0, dt, 
			  0, 0, 1, 0, 
			  0, 0, 0, 1;

			  
   // update Q matrix using noise_ax = 9 and noise_ay = 9
   float nax = 9.0; // noise_ax
   float nay = 9.0; // noise_ay
   float dt2 = dt*dt;
   float dt3 = dt2*dt;
   float dt4 = dt3*dt;
   
   ekf_.Q_ << dt4/4*nax, 0,         dt3/2*nax, 0,
              0,         dt4/4*nay, 0,         dt3/2*nay,
			  dt3/2*nax, 0,         dt2*nax,   0,
              0,         dt3/2*nay, 0,         dt2*nay;

   cout << "fusionekf(138): current ekf_.x_ =\n" << ekf_.x_ << endl;

   cout << "fusionekf(125): ############## PREDICT State x' and P'  ###########" << endl;
   cout << "fusionekf(127): timestamp_|previous_timestamp_|delta=" << measurement_pack.timestamp_ << "|" << previous_timestamp_ <<"|"<< measurement_pack.timestamp_-previous_timestamp_<< endl;
   
   ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	VectorXd raw_measurement(3);
	float rho    = measurement_pack.raw_measurements_(0); // rho_measured
	float phi    = measurement_pack.raw_measurements_(1); // phi_measured
	float rhodot = measurement_pack.raw_measurements_(2); // rhodot_measured
	  
	raw_measurement << rho, phi, rhodot;
	
	ekf_.R_ = MatrixXd(3,3);
	ekf_.R_ << R_radar_;

	//FusionEKF::print_EKF("FusionEKF(195)");
	cout << "FustionEKV(205) ekf_.x_=" << endl << ekf_.x_ << endl;
	cout << "FustionEKV(205) ekf_.P_=" << endl << ekf_.P_ << endl;
	cout << "FustionEKV(205) ekf_.F_(0,2)=" << ekf_.F_(0,2) << endl;
	cout << "FustionEKV(205) ekf_.H_.size=" << ekf_.H_.size() << endl;
	cout << "FustionEKV(205) ekf_.R_.size=" << ekf_.R_.size() << endl;
	cout << "FustionEKV(205) ekf_.Q_=" << endl << ekf_.Q_ << endl;
    cout << "fusionekf(216): ############## Radar UPDATE State x and P  ###########" << endl;
 
    ekf_.UpdateEKF(raw_measurement);
  
  } else {
    // Laser updates
	VectorXd raw_measurement(2);
	float px = measurement_pack.raw_measurements_(0); // x_measured
	float py = measurement_pack.raw_measurements_(1); // y_measured
	
	raw_measurement << px, py;  // for vx and vy, use the previous vx vy from x state

	ekf_.H_ = MatrixXd(2,4);
	ekf_.H_ << H_laser_;
	ekf_.R_ = MatrixXd(2,2);
	ekf_.R_ << R_laser_;

    cout << "FustionEKV(235) ekf_.x_=" << endl << ekf_.x_ << endl;
	cout << "FustionEKV(235) ekf_.P_=" << endl << ekf_.P_ << endl;
	cout << "FustionEKV(235) ekf_.F_(0,2)=" << ekf_.F_(0,2) << endl;
	cout << "FustionEKV(235) ekf_.H_.size=" << ekf_.H_.size() << endl;
	cout << "FustionEKV(235) ekf_.R_.size=" << ekf_.R_.size() << endl;
	cout << "FustionEKV(235) ekf_.Q_=" << endl << ekf_.Q_ << endl;
    cout << "fusionekf(216): ############## Lidar UPDATE State x and P  ###########" << endl;
  	
	ekf_.Update(raw_measurement);
  }

  // update previous_timestamp with current timestamp after all update is done
  previous_timestamp_ = measurement_pack.timestamp_; // timestamp_measured in microsecond unit;
  
  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
} // FusionKF::ProcessMeasurement
