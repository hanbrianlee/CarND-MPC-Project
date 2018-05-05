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

auto nowtime = std::chrono::high_resolution_clock::now();

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    //processing time measurement
//    auto begin = std::chrono::high_resolution_clock::now();
//    cout << "h nowtime: " << std::chrono::duration_cast<std::chrono::milliseconds>(begin - nowtime).count() << "ms" << endl;
//    nowtime = begin;

    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
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
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          //cout << "psi: " << psi << ", py: " << py << std::endl;

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // Need Eigen vectors for polyfit
          Eigen::VectorXd xwaypoints(ptsx.size());
          Eigen::VectorXd ywaypoints(ptsx.size());

          // create rotation matrix
          Eigen::Matrix2d M_rot;
          M_rot << cos(psi), sin(psi),
                   -sin(psi), cos(psi);
          for (int i = 0; i < ptsx.size(); i++)
          {
            Eigen::Vector2d p_waypoint(ptsx[i] - px, ptsy[i] - py); // translate
            Eigen::Vector2d p_waypoint_t = M_rot * p_waypoint; // rotate
            xwaypoints[i] = p_waypoint_t(0);
            ywaypoints[i] = p_waypoint_t(1);
          }

          // Transform the points to the vehicle's perspective. It's possible to smooth out driving by making the polyfit on the waypoints smoother.
//          for (unsigned int i = 0; i < ptsx.size(); i++) { //for every ptsx and ptxy received:
//            double dx_car_perspective = ptsx[i] - px;
//            double dy_car_perspective = ptsy[i] - py;
//            //because the waypoints provided by lake_track_waypoints.csv are fixed regardless of the heading angle of the
//            //vehicle is, it's necessary to take into account the heading angle (absolute angle from the map coordinates) when calculating the waypoints.
//            xwaypoints[i] = dx_car_perspective * cos(-psi) - dy_car_perspective * sin(-psi);
//            ywaypoints[i] = dx_car_perspective * sin(-psi) + dy_car_perspective * cos(-psi);
//          }

          // Fits a polynomial to the above x and y coordinates in car perspective to the desired order
//          auto coeffs = polyfit(xwaypoints, ywaypoints, 1); //use first order
//          auto coeffs = polyfit(xwaypoints, ywaypoints, 2); //use second order
          auto coeffs = polyfit(xwaypoints, ywaypoints, 3); //use third order

          //find out the initial cte for this MPC control instance  (every time step, the initial state will be as if the car is at 0,0 position with 0 degree angle, and is trying to figure out where to go)
          double cte = polyeval(coeffs, 0); //evaluated at the current vehicle's position which is at x = 0

          //figure out the initial epsi (error in angle) for this MPC control instance (every time step, the initial state will be as if the car is at 0,0 position with 0 degree angle, and is trying to figure out where to go)
          double epsi = -atan(coeffs[1]); //0 degree (basically the direction where the car is facing at the moment. x axis is in the direction of vehicle's forward movement, and y axis is on the left side. basically cartesian coordinate rotated 90 degrees counter-clockwise


          //define the initial state for the MPC instance (every time step, the initial state will be as if the car is at 0,0 position with 0 degree angle, and is trying to figure out where to go)
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi; //just dumb simple initialization of state

          //chose not to use this. The problem is that the following assumes the vehicle continues on a straight line for the next dt cycle, which is wrong
          //take into account 1 cycle of processing latency(approx 15~40ms) and data transfer delay(100ms)
          //h message update rate is very fast (3~4ms) so will be ignored
//          double pred_px = 0.0 + v*cos(0)*dt;
//          const double pred_py = 0.0 + v*sin(0)*dt;
//          double pred_psi = 0.0 + ((v / Lf) * -delta * dt);
//          double pred_v = v + a * dt;
//          double pred_cte = cte + v * sin(epsi) * dt;
//          double pred_epsi = epsi + ((v / Lf) * -delta * dt);

          //solve using MPC class solve method
          auto vars = mpc.Solve(state,coeffs);

          //vars[0] and vars[1] are returned by mpc.solve function with delta and a values. Assign them accordingly.
          double steer_value = vars[0]  / (deg2rad(25));
          double throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value; //negative has to be applied due to simulator (negative value: left turn, positive value: right turn)
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
//          vector<double> mpc_x_vals = {state[0]}; //the very initial points can be estimated as such
//          vector<double> mpc_y_vals = {state[1]}; //the very initial points can be estimated as such
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (unsigned int i = 2; i < vars.size(); i+=2) { //skip index 0 and 1 because they are mpc solved delta and a
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i+1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
//          const unsigned int polysize = 2.5;

//          for (unsigned int i = 1; i < N*polysize; i++) {
//            next_x_vals.push_back(i*polysize);
//            next_y_vals.push_back(polyeval(coeffs, i*polysize));
//          }

          for (unsigned int i = 0; i < xwaypoints.size(); i++) {
            next_x_vals.push_back(xwaypoints[i]);
            //next_y_vals.push_back(ywaypoints[i]);
            next_y_vals.push_back(polyeval(coeffs, xwaypoints[i]));
            //increase resolution by twice for smoother display of the waypoint line
            if(i != (xwaypoints.size() - 1)) //prevent addressing overflow index
            {
              next_x_vals.push_back((xwaypoints[i] + xwaypoints[i+1])/2); //midpoint between waypoints
              next_y_vals.push_back(polyeval(coeffs, (xwaypoints[i] + xwaypoints[i+1])/2)); //midpoint between waypoints
            }
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          std::cout << "delta: " << delta << ", cte: " << cte <<", epsi: " << epsi <<", a: " << a << ", steer: " << steer_value << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
//          auto end = std::chrono::high_resolution_clock::now();
//          std::cout << "processing time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() << "ms" << std::endl;

          this_thread::sleep_for(chrono::milliseconds(100)); //this characterizes data transfer delay (i.e. CAN or ethernet or w/e medium)
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
