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
using namespace std;
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
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
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

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // These are the waypoints received from the car simulator
          // ptsx and ptsy are the coordinates for next 6 planned path waypoints
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          
          // Car's current position, orientation, and speed
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          v *= 0.447;

          //Need some more vars to use in anti-latency calc below
          double steer_angle = j[1]["steering_angle"];
          double throttle = j[1]["throttle"];


          // Transform waypoints from map to car coordinates - i.e. set car at origin and 0 angle
          // this helps with fitting the polynom and also simplifies epsi calculation
          
          const double sin_psi = sin(0-psi);
          const double cos_psi = cos(0-psi);

          Eigen::VectorXd car_ptsx(6);
          Eigen::VectorXd car_ptsy(6);

          for (int i = 0; i < ptsx.size(); i++)
          {
            double shifted_x = ptsx[i] - px;
            double shifted_y = ptsy[i] - py;

            car_ptsx[i] = shifted_x * cos_psi - shifted_y * sin_psi;
            car_ptsy[i] = shifted_y * cos_psi + shifted_x * sin_psi;

          }

          // Fit waypoints to a polynomial to get a continuous curve for planned path
          auto coeffs = polyfit(car_ptsx, car_ptsy, 3);
          //cout<<"DEBUG - coeffs: "<<endl<<coeffs<<endl;

          //Calculate cross-track-error and orientation error angle (epsi)
          //CTE is approx value of our polynomial at x=0, following the transformation
          //Orientation error calc is dramatically simplified as both psi and px are 0
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          //Dealing with latency
          //Bring the vehicle state fowrard by the delay duration

          const double latency = 0.1; //reflect 100ms

          double px_lat = v * latency;
          double py_lat = 0;
          double psi_lat = - v * steer_angle * DT / LF;
          double v_lat = v + throttle * DT;
          double cte_lat = cte + v * sin(epsi) * DT;
          double epsi_lat = epsi + psi_lat; 



          // Initiate the state vector
          // Set all values
          Eigen::VectorXd state(6);
          state << px_lat, py_lat, psi_lat, v_lat, cte_lat, epsi_lat;
          //state << 0, 0, 0, v, cte, epsi;
          

          /*const double px_act = v * DT;
          const double py_act = 0;
          const double psi_act = - v * steer_angle * DT / LF;
          const double v_act = v + throttle * DT;
          const double cte_act = cte + v * sin(epsi) * DT;
          const double epsi_act = epsi + psi_act; 
          Eigen::VectorXd state(6);
          state << px_act, py_act, psi_act, v_act, cte_act, epsi_act;
          */


          // Pass the current state and polynomial coeffs to the solver
          // get back vector with next steering inputs (steering angle and acceleration)
          auto mpc_solve = mpc.Solve(state, coeffs);

         /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // Initialize the control variables, and extract from vector of results returned by the solver
     
          double steer_value = -mpc_solve[0]/ deg2rad(25);
          double throttle_value = mpc_solve[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value * LF;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals = mpc.solution_ptsx;
          vector<double> mpc_y_vals = mpc.solution_ptsy;


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          //They will be calculated from the polynomial
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          const double step_size = 2.0;
          const int num_points = 20;

          for(int i = 0; i < num_points; i++)
          {
            next_x_vals.push_back(i * step_size);
            next_y_vals.push_back(polyeval(coeffs, i * step_size));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
