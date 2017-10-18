#include "kalman_filter.h"
#include <math.h>
#include <iostream>

#define PI 3.14159265

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;    
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd z_pred = H_ * x_; //(2x1) = (2x4)*(4*1)
	VectorXd y = z - z_pred; //(2x1)
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	// convert cartesian to polar
  	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	VectorXd h_x(3);
	h_x(0) = sqrt(px*px+py*py);
	h_x(1) = atan2(py,px); //arctan function
	h_x(2) = (px*vx+py*vy)/h_x(0);
	
	VectorXd z_pred = h_x; //(3x1) 
	VectorXd y = z - z_pred; //(3x1)
	//adjust the phi angle so it's between -pi and pi
  	while(y(1)>PI or y(1) < -PI){
    	if(y(1)>PI){
      		y(1) = y(1)-2*PI;
    	}
    	else{
      		y(1) = y(1)+2*PI;
    	}    
  	}

	MatrixXd Ht = H_.transpose();//(4x3)
	MatrixXd S = H_ * P_ * Ht + R_;//(3x3) = (3x4)*(4x4)*(4x3)+(3*3)
	MatrixXd Si = S.inverse();//(3x3) 
	MatrixXd PHt = P_ * Ht;//(4x3) = (4x4)*(4x3)
	MatrixXd K = PHt * Si;//(4x3) = (4x3)*(3x3)
	//new estimate
	x_ = x_ + (K * y);//(4x1) = (4x1) + (4x3)*(3x1)
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;//(4x4) = (4x4)-(4x3)*(3x4)

}
