#include <math.h>
#include <iostream>
#include "twiddle.h"

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
Twiddle::Twiddle(int max_iter_, double max_cte_, vector<double> params_, double dp_tol_, vector<double> init_dp_, double min_speed_, 
	vector<bool> tunable_, int max_iter_no_movement_, double sum_dp_) : max_iter(max_iter_), max_cte(max_cte_), dp_tol(dp_tol_), 
	min_speed(min_speed_), max_iter_no_movement(max_iter_no_movement_), sum_dp(sum_dp_)
{
	params = params_;
	dp = init_dp_;
	curr_stage = 0;
	tunable = tunable_;
	curr_i = -1;
	if (sum_dp > 0)
	    curr_i = next_i();
	min_i = curr_i;
	iter_no_movement = 0;
}

/**
* reset Decide if the test of the current values of parameters should be reset
* @param curr_iter - current iteration of the test
* @param curr_cte - current cross-track error
* @param speed - current speed
*/
bool Twiddle::reset(int curr_iter, double curr_cte, double speed)
{
	if (speed < min_speed)
		iter_no_movement++;
	else
		iter_no_movement = 0;

	// reset if any of the following conditions holds:
	// 1) the test runs for more that max_iter iterations
	// 2) cross-track error fromt he last iteration is too large
	// 3) car didn't move in the last max_iter_no_movement iterations
	return curr_iter >= max_iter || fabs(curr_cte) >= max_cte || (iter_no_movement >= max_iter_no_movement);
}

/**
* nextValues Generate next values of parameters that should be tested. Returns true if the new values of parameters were generated, otherwise returns false. 
* @param params - vector of the current values of parameters. The new values of parameters are also placed into this vector. 
* @param err - error from the current iteration.
* @param iter - number of iterations completed in the current run
*/
bool Twiddle::nextValues(vector<double>& params, double err, double iter)
{
	// check if twiddle should stop generating new values
	if (sum_dp < dp_tol && curr_i == min_i)
		return false;

	// print error and the number of iterations completed in the current run
	std::cout << "cte = " << err << " " << " iter " << iter << std::endl;

	// finding new values of partameters is split into 3 stages
	// stage 0 - first run of the function 
	// stage 1 - checks result of the run when the value of the current active parameter was increased, choose the next values to test
	// stage 2 - checks result of the run when the value of the current active parameter was decreased, choose the next values to test
	switch (curr_stage) {
	
	case 0: // first run of the function
		// current parameters are the best ones found so far
		best_err = err;
		best_params = params;
		best_iter = iter;

		std::cout << "best params " << best_params[0] << " " << best_params[1] << " " << best_params[2] << " "
			                        << best_params[3] << " " << best_params[4] << " " << best_params[5] << " best_err " << best_err << " " << std::endl;

		// increase the value of the active parameter 
		params[curr_i] += dp[curr_i];
		curr_stage = 1;
		return true;

	case 1:  // checks result of the run when the value of the current active parameter was increased, choose the next values to test
		// check if the current values of parameters are better that all previously tested ones 
		if ((best_iter < max_iter && iter > best_iter) ||  // with previous values the car could not complete the full run, with the 
														   // current values the car could drive longer than with all previously tested values
			(best_iter < max_iter && iter == best_iter && err < best_err) || // with previous values the car could not complete the full run, 
																			 // with the current values the car completed exactly the same number
																			 // of iterations as the previously longest run, the error of the 
																			 // current values is smaller the error of the previous parameters 
																			 // values that generated the longest run
			(best_iter >= max_iter && err < best_err && iter >= max_iter)) { // with the current and the best previous values the car complete the full run,  
																			 // the error of the current values is the smallest one amongst the errors of parameter
																			 // values that could complete the full run
			// current parameters are the best ones found so far
			best_err = err;
			best_params = params;
			best_iter = iter;

			std::cout << "best params " << best_params[0] << " " << best_params[1] << " " << best_params[2] << " best_err " << best_err << std::endl;

			// adjust sum of the values of direction vector
			sum_dp = sum_dp - fabs(dp[curr_i]) + fabs(dp[curr_i]) * 1.1;

			// increase the direction step of the current active parameter
			// this step size will be used next time this parameter will be tuned
			dp[curr_i] *= 1.1;

			// find the next active parameter
			curr_i = next_i();

			// check if twiddle should stop generating new values of parameters
			if (curr_i == min_i && sum_dp < dp_tol) 
			    return true;   // we could also return false here. We are returning true to have same upstream processing as when we are returning from case 2.

			// increase the value of the active parameter 
			params[curr_i] += dp[curr_i];
		}
		else {
			// current values of parameters are not the best ones  
			// decrease the value of the active parameter
			params[curr_i] -= 2 * dp[curr_i];
			curr_stage = 2;
		}
		return true;

	case 2: // checks result of the run when the value of the current active parameter was increased, choose the next values to test
		if ((best_iter < max_iter && iter > best_iter) ||  // with previous values the car could not complete the full run, with the 
			                                               // current values the car could drive longer than with all previously tested values
			(best_iter < max_iter && iter == best_iter && err < best_err) || // with previous values the car could not complete the full run, 
			                                                                 // with the current values the car completed exactly the same number
			                                                                 // of iterations as the previously longest run, the error of the 
			                                                                 // current values is smaller the error of the previous parameters 
			                                                                 // values that generated the longest run
			(best_iter >= max_iter && err < best_err && iter >= max_iter)) { // with the current and the best previous values the car complete the full run,  
			                                                                 // the error of the current values is the smallest one amongst the errors of parameter
			                                                                 // values that could complete the full run
			// current parameters are the best ones found so far
			best_err = err;
			best_params = params;
			best_iter = iter;

			std::cout << "best params " << best_params[0] << " " << best_params[1] << " " << best_params[2] << " "
				      << best_params[3] << " " << best_params[4] << " " << best_params[5] << " " << " best_err " << best_err << std::endl;

			// adjust sum of the values of direction vector
			sum_dp = sum_dp - fabs(dp[curr_i]) + fabs(dp[curr_i]) * 1.1;

			// increase the direction step of the active parameter
			// this step size will be used next time this parameter will be tuned
			dp[curr_i] *= 1.1;
		}
		else {
			// current values of parameters are not the best ones  
			// restore the value of the active parameter
			params[curr_i] += dp[curr_i];

			// adjust sum of the values of direction vector
			sum_dp = sum_dp - fabs(dp[curr_i]) + fabs(dp[curr_i]) * 0.9;

			// decrease the direction step of the active parameter
			// this step size will be used next time this parameter will be tuned
			dp[curr_i] *= 0.9;
		}

		// find the next active parameter
		curr_i = next_i();

		// check if twiddle should stop generating new values of parameters
		if (curr_i == min_i && sum_dp < dp_tol) {
			params = best_params;
			return true;
		}

		// increase the value of the active parameter 
		params[curr_i] += dp[curr_i];
		curr_stage = 1;
		return true;
	}
}

/**
* next_i Return index of the next parameter that should be tuned
*/
int Twiddle::next_i()
{
	do {
		curr_i = (curr_i + 1) % params.size();
	} 
	while (tunable[curr_i] == false);
	return curr_i;
}