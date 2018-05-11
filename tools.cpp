#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  unsigned int i;
  	
  if((estimations.size() != ground_truth.size()) || (estimations.size() == 0)){
  	cout << "Invalid Estimation and Ground Truth sizes" << endl;
  	return rmse;
  }
    
  for(i=0;i<estimations.size();i++){
  	
  	VectorXd residual = estimations[i] - ground_truth[i];
  	residual = residual.array()*residual.array();
  	rmse += residual;
 
  }
  
  rmse = rmse/estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  float px = x_state[0];
  float py = x_state[1];
  float vx = x_state[2];
  float vy = x_state[3];
  float rho = sqrt(px*px+py*py);
  float rho_2 = rho*rho;
  float rho_3by2 = rho_2*rho;
  
  
  MatrixXd H_j(3,4);

  if(fabs(rho) < 0.000001){
    std::cout << "Function CalculateJacobian() has Divide by Zero" << std::endl;
    return H_j;
  }						  
  
  H_j << px/rho, py/rho, 0, 0,
  		-py/rho_2, px/rho_2, 0, 0,
		  py*(vx*py-vy*px)/rho_3by2, px*(vy*px-vx*py)/rho_3by2, px/rho, py/rho;  						  
						  
  return H_j;
  
}
