#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>

using namespace std;

class Twiddle {
public:

	/**
	* Constructor
	* @param max_iter_ - maximum number of iterations to test candidate values of parameters
	* @param max_cte_ - maximum allowable cte. The test of candidate values of parameters is stopped if cte exceeds max_cte
	* @param params_ - initial values of parameters
	* @param dp_tol_ - minimal sum of values of the direction vector to continue optimization of parameters
	* @param init_dp_ - initial direction vector for twiddle
	* @param min_speed_ - minimum allowable speed. If the car drives slower than this speed twiddle suspects that the car is stuck
	* @param tunable_ - vector of indicators if a parameter needs to be tuned
	* @param max_iter_no_movement_ - if the car has max_iter_no_movement iterations with the speed less than min_speed then twiddle
	*                                decides that the car is stuck and the current run is stopped
	* @param sum_dp_ - sum of values in the direction vector
	*/
	Twiddle(int max_iter_, double max_cte_, vector<double> params_, double dp_tol_, vector<double> init_dp_, double min_speed_, 
		    vector<bool> tunable_, int max_iter_no_movement_, double sum_dp_);

	/**
	* reset Decide if the test of the current values of parameters should be reset
	* @param curr_iter - current iteration of the test
	* @param curr_cte - current cross-track error
	* @param speed - current speed
	*/
	bool reset(int curr_iter, double curr_cte, double speed);

	/**
	* nextValues Generate next values of parameters that should be tested. Returns true if the new values of parameters were generated, otherwise returns false. 
	* @param params - vector of the current values of parameters. The new values of parameters are also placed into this vector.
	* @param err - error from the current iteration.
	* @param iter - number of iterations completed in the current run
	*/
	bool nextValues(vector<double>& params, double err, double iter);

private:
	int max_iter;
	double max_cte;
	int curr_i;
	int curr_stage;
	double best_err;
	int best_iter;
	vector<double> best_params;
	vector<double> params;
	vector<double> dp;
	double sum_dp;
	double dp_tol;
	double min_speed;
	vector<bool> tunable;
	int min_i;
	int iter_no_movement;
	int max_iter_no_movement;

	/**
	* next_i Return index of the next parameter that should be tuned
	*/
	int next_i();
};

#endif
