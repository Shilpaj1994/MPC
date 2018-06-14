#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << std::endl;
	//Debugging
	//std::cout << "pit stop 1" << std::endl;
	//
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
	  //Debugging
	  //std::cout << "pit stop 2" << std::endl;
	  //
      if (s != "") {
		//Debugging
		//std::cout << "pit stop 3" << std::endl;
		//
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
		  double steering_angle = j[1]["steering_angle"];
		  double throttle = j[1]["throttle"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

		  //ptsx and ptsy are given to us by the simulator in the global coordinates.
		  //These values are transformed to the car coordinates by rotation and translation
		  for (unsigned int i = 0; i < ptsx.size(); i++)
		  {
			  double shift_x = ptsx[i] - px;
			  double shift_y = ptsy[i] - py;

			  ptsx[i] = (shift_x*cos(0 - psi) - shift_y*sin(0 - psi));
			  ptsy[i] = (shift_x*sin(0 - psi) + shift_y*cos(0 - psi));
		  }

		  //number of points used to find coefficients
		  int num_ptsfor_coeffs = 6;

		  //transforming the ptsx and ptsy into Eigen vectors, so that they can be used in polyfit
		  double* ptrx = &ptsx[0];
		  Eigen::Map<Eigen::VectorXd> eigen_ptsx(ptrx, num_ptsfor_coeffs);

		  double* ptry = &ptsy[0];
		  Eigen::Map<Eigen::VectorXd> eigen_ptsy(ptry, num_ptsfor_coeffs);


		  //fitting polynomials on the waypoints to find the coeffecients of the waypoints in car coordinates
		  Eigen::VectorXd coeffs = polyfit(eigen_ptsx, eigen_ptsy, 3);


		  double cte = polyeval(coeffs, 0);
		  //check the calculation
		  //double epsi = psi - atan(3 * coeffs[3] * px*px + 2 * coeffs[2] * px + coeffs[1]);
		  double epsi = -atan(coeffs[1]);

		  //Latency time value
		  double latency = 0.1;

		  // This is the length from front to CoG that has a similar radius.
		  const double Lf = 2.67;

		  //These are the state values at the actual time the actuation commands reach.
		  //That is these are the state values after the latency time is considered
		  double px_act = v * latency;
		  double py_act = 0;
		  double psi_act = -v * steering_angle * latency / Lf;		  
		  double cte_act = cte + v * sin(epsi) * latency;
		  double epsi_act = epsi + psi_act;
		  double v_act = v + throttle * latency;
		  
		  // the state values after the latency is passed to mpc.Solve
		  Eigen::VectorXd initial_state(6);
		  initial_state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;


		  //values from mpc.Solve include first actuator commands, and the mpc predicted x, and y values
		  //The mpc predicted x, and y values are represented by a green line in the simulator.
		  vector<double> values = mpc.Solve(initial_state, coeffs);

          double steer_value = values[0];
          double throttle_value = values[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
		  msgJson["steering_angle"] = -steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
		  size_t N = 20;
		  for (unsigned int i = 2; i < 2 + N; i++)
		  {
			  mpc_x_vals.push_back(values[i]);
		  }

		  for (unsigned int i = 2 + N; i < 2 + 2*N; i++)
		  {
			  mpc_y_vals.push_back(values[i]);
		  }


          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          //Display the waypoints/reference line
		  int num_points = 30;
          vector<double> next_x_vals(num_points);
          vector<double> next_y_vals(num_points);

		  double poly_inc = 2.0;
		  
		  for (int i = 1; i < num_points; i++)
		  {
			  next_x_vals.push_back(poly_inc*i);
			  next_y_vals.push_back(polyeval(coeffs, poly_inc*i));
		  }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          
		  
		  // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[  \"manual\",{}]";
		//Debugging
		//std::cout << " pit stop " << std::endl;
		//
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

