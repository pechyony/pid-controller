#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "twiddle.h"

// for convenience
using json = nlohmann::json;
using namespace std;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

void print_usage()
{
	cout << "Usage" << endl;
	cout << "-----" << endl;
	cout << "Run with defaul values of parameters:" << endl;
	cout << "    CarND - PID - Controller - Project.exe" << endl;
	cout << "Run with fixed values of parameters:" << endl;
	cout << "    CarND - PID - Controller - Project.exe run <max throttle> <Kp> <Ki> <Kd> <s_Kp> <s_Ki> <s_Kd>" << endl;
	cout << "Tune parameters to optimize a given metric, starting from given values:" << endl;
	cout << "    CarND - PID - Controller - Project.exe tune <max throttle> <Kp> <Ki> <Kd> <s_Kp> <s_Ki> <s_Kd> <tune_Kp> <tune_Ki> <tune_Kd> <tune_s_Kp> <tune_s_Ki> <tune_s_Kd>[cte | speed]" << endl;
}

int main(int argc, char* argv[])
{
  uWS::Hub h;
  
  PID pid;   // steering controller
  PIDAbs pid_v; // throttle controller

  // constants to be passed to twiddle
  const int max_iter = 10000; // maximum number of iterations to test candidate values of parameters
  const double max_cte = 3.14; // 1.57 maximum allowable cte. The test of candidate values of parameters is stopped if cte exceeds max_cte
  const double min_speed = 0.02; // minimum allowable speed. If the car drives slower than this speed twiddle suspects that the car is stuck
  const int max_iter_no_movement = 10; // if the car has max_iter_no_movement iterations with the speed less than min_speed then twiddle
                                       // decides that the car is stuck and the current run is stopped

  // default values of tunable parameters
  double max_throttle = 0.8; 
  double Kp = 1; 
  double Ki = 0.01;
  double Kd = 40; 
  double s_Kp = 1;
  double s_Ki = 0.001;
  double s_Kd = 0.01;

  // indicator if a parameter needs to be tuned
  bool tune_Kp = false;
  bool tune_Ki = false;
  bool tune_Kd = false;
  bool tune_s_Kp = false;
  bool tune_s_Ki = false;
  bool tune_s_Kd = false;

  string mode; // running mode, "run" or "tune"
  string tune_metric; // metric to be optimized, "cte" or "speed"

  // Initialize pid variable
  if (argc == 9 || argc == 16) {
	  // "run" or "tune" mode, get initial values of parameters
	  mode = argv[1];
	  max_throttle = stod(argv[2]);
	  Kp = stod(argv[3]);
	  Ki = stod(argv[4]);
	  Kd = stod(argv[5]);
	  s_Kp = stod(argv[6]);
	  s_Ki = stod(argv[7]);
	  s_Kd = stod(argv[8]);
	  if (argc == 16) {
		  // "tune" mode, get the tuning metric and the indicators what parameters should be optimized	
		  tune_Kp = stoi(argv[9]);
		  tune_Ki = stoi(argv[10]);
		  tune_Kd = stoi(argv[11]);
		  tune_s_Kp = stoi(argv[12]);
		  tune_s_Ki = stoi(argv[13]);
		  tune_s_Kd = stoi(argv[14]);
		  tune_metric = argv[15];
	  }
  }
  else
	  if (argc == 1)
		  // running with default values of parameters
		  mode = "run";
  
  // check correctness of the command line parameters
  if (!(argc == 1 || argc == 9 || argc == 16) || !(mode == "tune" || mode == "run") || 
	  (argc == 16 && (tune_metric != "cte" && tune_metric != "speed"))) {
	  print_usage();
	  exit(-1);
  }
  
  // initialize PID controllers
  pid.Init(Kp, Ki, Kd);
  pid_v.Init(s_Kp, s_Ki, s_Kd);

  // initialize a vector of parameter values to be used by twiddle
  vector<double> params(6);
  params[0] = Kp;
  params[1] = Ki;
  params[2] = Kd;
  params[3] = s_Kp;
  params[4] = s_Ki;
  params[5] = s_Kd;

  // initialize a vector of indicators if a parameter needs to be tuned, to be used by twiddle
  // in "run" mode all entries of this vector are "false"
  vector<bool> tunable(6);
  tunable[0] = tune_Kp;
  tunable[1] = tune_Ki;
  tunable[2] = tune_Kd;
  tunable[3] = tune_s_Kp;
  tunable[4] = tune_s_Ki;
  tunable[5] = tune_s_Kd;

  // compute initial direction vector for twiddle 
  // in "run" mode this vector might contain garbage
  vector<double> init_dp(6);
  double sum_dp = 0; // sum of values in the direction vector
  for (int i = 0; i < 6; i++) {
	  if (tunable[i]) {
		  init_dp[i] = params[i] / 2; 
		  if (init_dp[i] == 0)
			  init_dp[i] = 0.1;     
		  sum_dp += init_dp[i];
	  }
  }
  const double dp_tol = sum_dp / 2;  // minimal sum of values of the direction vector to continue optimization of parameters

  // initialize twiddle
  Twiddle twiddle(max_iter, max_cte, params, dp_tol, init_dp, min_speed, tunable, max_iter_no_movement, sum_dp);

  int iter = 0;             // current iteration
  double total_err = 0;     // total accumulated error
  double total_err_cte = 0; // total accumulated squared cte
  double total_speed = 0;   // total accumulated normalized throttle values
  bool is_reset = false;    // indicator if we need to stop the current run and proceed to the next values of parameters

  h.onMessage([&pid, &pid_v, &iter, &twiddle, &Kp, &Ki, &Kd, &s_Kp, &s_Ki, &s_Kd, &params, &total_err, &mode, &is_reset, 
	           &max_throttle, &tune_metric, &total_err_cte, &total_speed](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
		  
          // j[1] is the data JSON object
          double cte = stod(j[1]["cte"].get<string>());
          double speed = stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());

		  // update steering angle
		  pid.UpdateError(cte);
          double steer_value = pid.TotalError();
          
		  // update throttle
		  pid_v.UpdateError(cte);
		  double throttle_value = max_throttle + pid_v.TotalError();
		  if (throttle_value < 0.01)
		  	  throttle_value = 0.01;

          // update metric value of the current values of parameters  
		  if (tune_metric == "cte") {
			  total_err += cte * cte;
			  total_err_cte = total_err;
		  }
		  else {
			  total_err_cte += cte * cte;
			  total_speed += speed;
			  total_err += cte * cte - throttle_value / max_throttle;
		  }
          iter++;

		  // send "drive" or "reset" message to simulator (depending on the current driving mode and errors)
          json msgJson;
		  string msg;
		  if (is_reset == true) {
			  // second reset message (because of bug in the simulator: the after the first reset message simulator sends a huge value of cte)
			  msg = "42[\"reset\",{}]";
			  iter = 0;
			  total_err = 0;
			  total_err_cte = 0;
			  total_speed = 0;
			  is_reset = false;
		  }
		  else if (mode == "tune" && twiddle.reset(iter, cte, speed) && twiddle.nextValues(params, total_err, iter)) {

			  // print more details about error 
			  cout << "RMSE of cte " << sqrt(total_err_cte/iter) << " average speed" << total_speed/iter << endl;

			  // finished testing the current values of parameters, restart the the simulator with the new values 
			  msg = "42[\"reset\",{}]";
			  iter = 0;
			  total_err = 0;
			  total_err_cte = 0;
			  total_speed = 0;

			  // initialize steering controller with the new values of parameters
			  Kp = params[0];
			  Ki = params[1];
			  Kd = params[2];
			  pid.Init(Kp, Ki, Kd);

			  // initialize speed controller with the new values of parameters
			  s_Kp = params[3];
			  s_Ki = params[4];
			  s_Kd = params[5];
			  pid_v.Init(s_Kp, s_Ki, s_Kd);
			  is_reset = true;

			  cout << "New params Kp " << Kp << " Ki " << Ki << " Kd " << Kd << " s_Kp " << s_Kp << " s_Ki " << s_Ki << " s_Kd " << s_Kd << endl;
		  }
		  else {
			  // continue driving with the current steering and throttle values
			  msgJson["steering_angle"] = steer_value;
			  msgJson["throttle"] = throttle_value; 
			  msg = "42[\"steer\"," + msgJson.dump() + "]";
		  }
		  
          ws -> send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws -> send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
	  const std::string s = "<h1>Hello world!</h1>";
	  if (req.getUrl().valueLength == 1)
	  {
		  res -> end(s.data(), s.length());
	  }
	  else
	  {
		  // i guess this should be done more gracefully?
		  res -> end(nullptr, 0);
	  }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) {
	  cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>* ws, int code, char *message, size_t length) {
	  ws -> close();
	  cout << "Disconnected" << endl;
  });

  int port = 4567;
  auto host = "127.0.0.1";
  if (h.listen(host, port))
  {
	  cout << "Listening to port " << port << endl;
  }
  else
  {
	  cerr << "Failed to listen to port" << endl;
	  return -1;
  }
  h.run();
}
