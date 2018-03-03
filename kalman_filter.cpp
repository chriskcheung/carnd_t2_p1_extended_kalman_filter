#include "kalman_filter.h"
#include "tools.h"
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd y  = z - H_*x_;
  cout << "kalman_filter(47) - y=\n" << y << endl;
  MatrixXd Ht = H_.transpose();
  cout << "kalman_filter(47) - Ht=\n" << Ht << endl;
  MatrixXd S  = H_*P_*Ht + R_;
  cout << "kalman_filter(47) - S=\n" << S << endl;
  MatrixXd Si = S.inverse();
  cout << "kalman_filter(47) - Si=\n" << Si << endl;
  MatrixXd K  = P_*Ht*Si;
  cout << "kalman_filter(47) - K=\n" << K << endl;
  
  // new state
  x_ = x_ + K*y;
  cout << "kalman_filter(47) - new L x=\n" << x_ << endl;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  //cout << "kalman_filter(47) - I=\n" << I << endl;
  P_ = (I - K*H_)*P_;
  cout << "kalman_filter(47) - new L P=\n" << P_ << endl;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  Tools tools;
  MatrixXd Hj = MatrixXd(3,4);
  // for Radar update, use predicted state x to calculate Jacobian
  Hj << tools.CalculateJacobian(x_);
  
  // Skip radar update if Hj is not valid
  if (Hj.size() == 0) {
	cout << "************************************************************" << endl;
	cout << "CalculateJacobian() - Error - divided by zero. skip updating" << endl;
	cout << "************************************************************" << endl;
    return; 
  }

  // for Radar update, use predicted state x to calculate h(x) instead of using Matrix H
  VectorXd h(3);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float d1 = sqrt(px*px + py*py);
  // Skip update if Hj is not valid
  float d2 = atan2(py,px);
  float d3 = d1 >= 0.00001 ? (px*vx + py*vy)/d1 : 0.1;
  cout << "************************************************************" << endl;
  cout << "kalman_filter(100) - d1|d2|d3=" << d1<<"|"<<d2<<"|"<<d3 << endl;
  cout << "************************************************************" << endl;

  h << d1, d2, d3;
  cout << "kalman_filter(115) - h=\n" << h << endl;
  MatrixXd y  = z - h;
  cout << "kalman_filter(118) - y=\n" << y << endl;

  // continue update state using H Jacobian matrix			
  // normalize phi, y(1) in this case to be within -pi to pi
  const double pi = 3.14159265358979323846;
  while(y(1) > pi){
    y(1) -= 2.*pi;
	cout << "kalman_filter(107) - y(1) > pi -> normalized to y(1)-2pi=" << y(1) << endl;
  }
  while(y(1) < -pi){
    y(1) += 2.*pi;
	cout << "kalman_filter(111) - y(1) < pi -> normalized to y(1)+2pi=" << y(1) << endl;
  }

  cout << "kalman_filter(118) - y=\n" << y << endl;
  MatrixXd Ht = Hj.transpose();
  cout << "kalman_filter(118) - Ht=\n" << Ht << endl;
  MatrixXd S  = Hj*P_*Ht + R_;
  cout << "kalman_filter(118) - S=\n" << S << endl;
  MatrixXd Si = S.inverse();
  cout << "kalman_filter(118) - Si=\n" << Si << endl;
  MatrixXd K  = P_*Ht*Si;
  cout << "kalman_filter(118) - K=\n" << K << endl;
  
  // new state
  x_ = x_ + K*y;
  cout << "kalman_filter(118) - new R x=\n" << x_ << endl;
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  //cout << "kalman_filter(118) - I=\n" << I << endl;
  P_ = (I - K*Hj)*P_;
  cout << "kalman_filter(118) - new R P=\n" << P_ << endl;
}
