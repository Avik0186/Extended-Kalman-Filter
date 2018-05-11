#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  VectorXd y = z - H_*x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_*Ht; 
  MatrixXd S = H_*PHt + R_;
  MatrixXd Si = S.inverse();
  
  //cout << "Si = " << Si << endl;
  
  MatrixXd K = PHt*Si;
 
  x_ = x_ + K*y;
  P_ = (I-K*H_)*P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  MatrixXd I=MatrixXd::Identity(x_.size(),x_.size());
  float rho = sqrt(x_[0]*x_[0]+x_[1]*x_[1]);
  float phi = atan2(x_[1],x_[0]);
  if(fabs(rho)<=0.000001){
  	rho = 0.000001;
  }
  float rho_dot = (x_[0]*x_[2] + x_[1]*x_[3])/rho;
  VectorXd h(3);
  h << rho, phi, rho_dot;
  
  VectorXd y = z - h;
    // normalize the angle between -pi to pi
  while(y(1) > M_PI){
    y(1) -= 2*M_PI;
  }

  while(y(1) < -M_PI){
    y(1) += 2*M_PI;
  }
  
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_*Ht;
  MatrixXd S = H_*PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt*Si;
  
  //cout << "Si = " << Si << endl;
  
  x_ = x_ + K*y;
  P_ = (I-K*H_)*P_;
  
}
