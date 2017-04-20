#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define PI 3.14159265

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

  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;
    
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  //Laser H matrix
  H_laser_ << 1,0,0,0,
        	0,1,0,0;

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
  	
  	cout << "Kalman Filter Initialization " << endl;

	//Initialize the state ekf_.x_
	ekf_.x_ = VectorXd(4);
	ekf_.x_ << 1, 1, 1, 1;
	
	// Initialize State covariance matrix P
  	ekf_.P_ = MatrixXd(4, 4);
  	ekf_.P_<< 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;
	
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
     float px = 0.0;
     float py = 0.0;
     float vx = 0.0;
     float vy = 0.0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    	
      /**
       Convert radar from polar to cartesian coordinates and initialize state.
      */
    	
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float rhodot = measurement_pack.raw_measurements_(2);

      px = rho * cos(phi);
      py = rho * sin(phi);
      vx = rhodot * cos(phi);
      vy = rhodot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    	
    	//initialize with position from file
      px = measurement_pack.raw_measurements_[0];
	  py = measurement_pack.raw_measurements_[1];
	  vx = 0;
	  vy = 0;
    }

    if(fabs(px) < .00001 && fabs(py) < .00001) {
        //skip this reading
        return;
    }
    

    //Initialize
    ekf_.x_ << px,py,vx,vy;
    cout << "EKF: " << endl;
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
   //compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;

	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	//Modify the F matrix so that the time is integrated
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, dt, 0,
		  0, 1, 0, dt,
		  0, 0, 1, 0,
		  0, 0, 0, 1;
	
	ekf_.Q_ = MatrixXd(4, 4);

	ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

   ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    ekf_.hx_ = VectorXd(3);
      
    float px = ekf_.x_[0];
    float py = ekf_.x_[1];
    float vx = ekf_.x_[2];
    float vy = ekf_.x_[3];
    
    float rho;
    float phi;
    float rhodot;

	// if px and py are small at any point, set phi and rhodot to zero
	if(fabs(px) < 0.0001 or fabs(py) < 0.0001){
        
        if(fabs(px) < 0.0001){
          px = 0.0001; //set px to 0.0001
        }

        if(fabs(py) < 0.0001){
          py = 0.0001;   //set py to 0.0001
        }
        
        rho = sqrt(px*px + py*py);
        phi = 0;	//set phi to zero
        rhodot = 0;	//set vel to zero
        cout << "set phi,rhodot zero" <<endl;
      } else {
        rho = sqrt(px*px + py*py);
        phi = atan2(py,px);
        //phi value should be between +-PI
        while(fabs(phi) > PI) {
            cout << "Adjusting phi " << phi << endl;
            if(phi < 0) {
                phi = phi + 2*PI;
            } else {
                phi = phi - 2*PI;
            }
        }
        rhodot = (px*vx + py*vy) /rho;
      }      
	
	ekf_.hx_ <<rho, phi, rhodot;
	ekf_.R_ = R_radar_;
	Tools tools;
	ekf_.H_ = tools.CalculateJacobian(ekf_.x_); //H_jacob is calculated
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
      /*
      //Test code to see what accuracy we get my using RADAR data as LASER
      
       float rho = measurement_pack.raw_measurements_(0);
       float phi = measurement_pack.raw_measurements_(1);
       float rhodot = measurement_pack.raw_measurements_(2);
      
      float px = rho * cos(phi);
      float py = rho * sin(phi);
      float vx = rhodot * cos(phi);
      float vy = rhodot * sin(phi);
      
      ekf_.R_ = R_laser_;
      ekf_.H_ = H_laser_;
      x_ = VectorXd(2);
      x_ << px,py;
      ekf_.Update(x_);
 */
  }
  else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
