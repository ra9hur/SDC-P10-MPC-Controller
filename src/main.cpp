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
    cout << "Data: " << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Way points in Global coordinates (data from GPS)
          // to be transformed to car coordinates
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          // Car measured state
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          //v *= 0.44704;      // Convert from mph to m/s

          double delta = j[1]["steering_angle"];
          double acceleration = j[1]["throttle"];

          // there will be a delay as the command propagates through the system. 
          // A realistic delay might be on the order of 100 milliseconds.
          // predict state in 100ms
          // Running a simulation using the vehicle model starting from the current state for the duration of the latency. 
          // Resulting state from the simulation is the new initial state for MPC.
          double latency = 0.1;
          double Lf = 2.67;
          px += v*cos(psi)*latency;
          py += v*sin(psi)*latency;
          psi += v*(-delta)/Lf*latency;         // negative sign included, to be consistent with psi direction
          v += acceleration*latency;

          Eigen::VectorXd car_ptsx(ptsx.size());
          Eigen::VectorXd car_ptsy(ptsx.size());

		      // convert to vehicle space
		      // Affine transformation. Translate to car coordinate system then rotate to the car's orientation
          // Ref: https://gamedev.stackexchange.com/questions/79765/how-do-i-convert-from-the-global-coordinate-space-to-a-local-space
          for (int i=0; i < ptsx.size(); i++) {
            double relativeX = ptsx[i] - px;
            double relativeY = ptsy[i] - py;

            // Counter-clockwise angles in the vehicle model are positive
            // However, counter-clockwise angles on the simulator steering command scale are negative
            car_ptsx[i] = relativeX*cos(-psi) - relativeY*sin(-psi);
            car_ptsy[i] = relativeX*sin(-psi) + relativeY*cos(-psi);
          }

          // Fit the polynomial (3rd order) to the waypoints
          auto coeffs = polyfit(car_ptsx, car_ptsy, 3);

          // Calculate cross track error
          double cte = polyeval(coeffs, px) - py;
          //double cte = polyeval(coeffs, 0);

          // Calculate the orientation error
          double epsi = psi - atan(coeffs[1] + 2*px*coeffs[2] + 3*coeffs[3]*pow(px,2));
          //double epsi = -atan(coeffs[1]);

          Eigen::VectorXd state(6);
          // Transformation results in vehicle's co-ordinates and yaw angle as zero
          state << 0, 0, 0, v, cte, epsi;

          auto vars = mpc.Solve(state, coeffs);

          /*
          * Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          // Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          //double steer_value = vars[0]/(0.436332*Lf);
          double steer_value = vars[0]/(deg2rad(25));
          std::cout << "Steering Angle: " << steer_value << std::endl;

          double throttle_value = vars[1];
          std::cout << "Throttle: " << throttle_value << std::endl;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (int i=2; i<vars.size(); i++) {
            if (i%2 == 0) {
              mpc_x_vals.push_back(vars[i]);
            }
            else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double poly_inc = 2.5;
          int num_points = 25;

          for (int i =1; i < num_points; i++) {
	          next_x_vals.push_back(poly_inc*i);
	          next_y_vals.push_back(polyeval(coeffs,poly_inc*i));
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does not actuate the commands instantly.
          //
          // there will be a delay as the command propagates through the system. 
          // A realistic delay might be on the order of 100 milliseconds.
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
        std::string msg = "42[\"manual\",{}]";
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
