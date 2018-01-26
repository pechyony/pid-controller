#include <math.h>
#include <iostream>
#include "PID.h"

using namespace std;

/**
* Constructor
*/
PID::PID() {}

/**
* Destructor.
*/
PID::~PID() {}

/**
* init Initialize controller
* @param Kp - value of Kp parameter
* @param Ki - value of Ki parameter
* @param Kd - value of Kd parameter
*/
void PID::Init(double Kp_, double Ki_, double Kd_) {
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;
	p_error = 0;
	i_error = 0;
	d_error = 0;
}

/**
* UpdateError Update parameters of PID controller given cross track error
* @param cte - cross-track error
*/
void PID::UpdateError(double cte) {
	double prev_cte = p_error;
	p_error = cte;
	i_error += cte;
	d_error = cte - prev_cte;
}

/**
* TotalError Calculate the total error of PID controller
*/
double PID::TotalError() {
	return -Kp*p_error-Ki*i_error-Kd*d_error;
}

/**
* TotalError Calculate the total error of PIDAbs controller.
*/
double PIDAbs::TotalError() {	
	return -Kp * fabs(p_error) - Ki * fabs(i_error) - Kd * d_error;
}

