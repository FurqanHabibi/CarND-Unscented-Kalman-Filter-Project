#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>
#define EPS 0.0000001 // A very small number

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  ///* initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  //std_a_ = 30;
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 30;
  std_yawdd_ = 0.5;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = n_x_ + 2;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    cout << "UKF initialization..." << endl;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      //Convert radar from polar to cartesian coordinates and initialize state.
      x_ << meas_package.raw_measurements_[0] * cos(meas_package.raw_measurements_[1]), meas_package.raw_measurements_[0] * sin(meas_package.raw_measurements_[1]), 0, 0, 0;
      P_ << std_radr_ * std_radr_, 0, 0, 0, 0,
            0, std_radr_ * std_radr_, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      //Initialize state.
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
      P_ << std_laspx_ * std_laspx_, 0, 0, 0, 0,
            0, std_laspy_ * std_laspy_, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    }

    // done initializing, no need to predict or update
    time_us_ = meas_package.timestamp_;

    is_initialized_ = true;
    cout << "UKF initialized!" << endl;

    // empty the NIS data files
    ofstream lidarNISFile("lidar-NIS.csv", ios::trunc);
    if (lidarNISFile.is_open()) {
      lidarNISFile << "A" << endl;
      lidarNISFile.close();
    }
    ofstream radarNISFile("radar-NIS.csv", ios::trunc);
    if (radarNISFile.is_open()) {
      radarNISFile << "A" << endl;
      radarNISFile.close();
    }

    return;
  }

  // Skip prediction and update if the incoming sensor is not used
  if (((meas_package.sensor_type_ == MeasurementPackage::RADAR) && !use_radar_) ||
      ((meas_package.sensor_type_ == MeasurementPackage::LASER) && !use_laser_)) {
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  //compute the time elapsed between the current and previous measurements
	float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
	time_us_ = meas_package.timestamp_;
  Prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    UpdateRadar(meas_package);
    cout << "z = " << meas_package.raw_measurements_ << endl;
    
  } else {
    // Laser updates
    UpdateLidar(meas_package);
  }

  // print the output
  cout << "x_ = " << x_ << endl << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  /*****************************************************************************
   *  Calculate Sigma Points
   ****************************************************************************/

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  MatrixXd term = std::sqrt(lambda_ + n_aug_) * L;
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i + 1)          = x_aug + term.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - term.col(i);
  }

  /*****************************************************************************
   *  Predict Sigma Points
   ****************************************************************************/

  //predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > EPS) {
        px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * (-cos(yaw + yawd*delta_t) + cos(yaw));
    }
    else {
        px_p = p_x + v*cos(yaw)*delta_t;
        py_p = p_y + v*sin(yaw)*delta_t;
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p += 0.5*delta_t*delta_t*cos(yaw)*nu_a;
    py_p += 0.5*delta_t*delta_t*sin(yaw)*nu_a;
    v_p += delta_t*nu_a;
    yaw_p += 0.5*delta_t*delta_t*nu_yawdd;
    yawd_p += delta_t*nu_yawdd;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  /*****************************************************************************
   *  Predict State Mean and Covariance
   ****************************************************************************/

  // set weights
  double weight_0 = lambda_/(lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i < 2 * n_aug_ + 1; ++i) {  //2n+1 weights
    double weight = 0.5/(n_aug_ + lambda_);
    weights_(i) = weight;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ += weights_(i) * x_diff * x_diff.transpose() ;
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  //set measurement dimension, lidar can measure px and py
  int n_z = 2;

  //measurement data
  VectorXd z = meas_package.raw_measurements_;

  //measurement matrix
	MatrixXd H = MatrixXd(n_z, n_x_);
	H <<  1, 0, 0, 0, 0,
			  0, 1, 0, 0, 0;
  
  //measurement covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R <<  std_laspx_ * std_laspx_, 0,
        0, std_laspx_ * std_laspx_;
  
  VectorXd z_pred = H * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H.transpose();
	MatrixXd S = H * P_ * Ht + R;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
	P_ = (I - K * H) * P_;

  //calculate NIS and write in text file
  double lidarNIS = y.transpose() * Si * y;
  ofstream lidarNISFile("lidar-NIS.csv", ios::app);
  if (lidarNISFile.is_open()) {
    lidarNISFile << lidarNIS << endl;
    lidarNISFile.close();
  }
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //measurement data
  VectorXd z = meas_package.raw_measurements_;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;


  /*****************************************************************************
   *  Update
   ****************************************************************************/
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sigma points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //calculate NIS and write in text file
  double radarNIS = z_diff.transpose() * S.inverse() * z_diff;
  ofstream radarNISFile("radar-NIS.csv", ios::app);
  if (radarNISFile.is_open()) {
    radarNISFile << radarNIS << endl;
    radarNISFile.close();
  }
}
