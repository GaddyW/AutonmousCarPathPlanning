#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  
  // variables to store last path
  vector<vector <double>> path_s;
  vector<vector <double>> path_d;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  //improve granularity of the map to 1 meter to improve frenet/cartesian conversions for JMT calculations
  tk::spline spline_x;
  tk::spline spline_y;
  tk::spline spline_dx;
  tk::spline spline_dy;
  
  spline_x.set_points(map_waypoints_s, map_waypoints_x);
  spline_y.set_points(map_waypoints_s, map_waypoints_y);
  spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
  spline_dy.set_points(map_waypoints_s, map_waypoints_dy);
  
  
  granular_map gran_map = get_map(spline_x, spline_y, spline_dx, spline_dy, max_s);
  
  //listen for messages and act
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s, &map_waypoints_dx,&map_waypoints_dy, &gran_map, &path_s, &path_d]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
                   
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          double timeframe = 0.02;
          double pathtime = 7.0;
          double target_velocity = 50 * 0.44704;
         
          //begin trajectory creation

          double start_s;
          double start_d;
          double end_d;
          double start_speed;
          double start_accel;
          bool change_behavior = false;
          double buff;  //variable to determine how much of the previous path to include in the subsquent path
          int prev_path_size = previous_path_y.size();
          
          if (prev_path_size == 0 ) {  //initialization when there is no path
            buff = 0;
            path_s.push_back({car_s, car_speed * 0.44704, 0});
            path_d.push_back({car_d, 0, 0});
          } else {
 			//remove from points that have been used from the s and d paths
            int elapsed_points = path_s.size() - previous_path_y.size();
            path_s.erase(path_s.begin(),path_s.begin()+elapsed_points);
            path_d.erase(path_d.begin(),path_d.begin()+elapsed_points);

            //determine how many new points need to be added.  If we're changing behavior, than keep the first 1/2 second of the previous path and then append new values.  If 
            //there is no change in behavior, then we can simply append to the end of the previous path

            if (change_behavior == true) {
              buff = .5/timeframe;
            } else {
              buff = prev_path_size-1;
            }
             
          }
          
          // Create Jerk Minimizing Trajectory coefficients
          int num_points_required = pathtime/timeframe - buff;
          double time_required = num_points_required*timeframe;
          vector<double> s_start = {path_s[buff][0], path_s[buff][1], path_s[buff][2]};
          vector<double> s_end = {path_s[buff][0] + (target_velocity+path_s[buff][1])/2*time_required, target_velocity, 0};
          vector<double> d_start = {path_d[buff][0], path_d[buff][1], path_d[buff][2]};
          vector<double> d_end = {6, 0, 0};
		  vector<double> s_coeffs = JMT(s_start, s_end, time_required);
          vector<double> d_coeffs = JMT(d_start, d_end, time_required);
		  
     
          
          double s, s_dot, s_dot_dot;
          double d, d_dot, d_dot_dot;
          double t;
          for(int i = 1; i < num_points_required ; ++i) {
    		t = i * timeframe;
            s         =   s_coeffs[0] +        s_coeffs[1] * t +     s_coeffs[2] * t*t +     s_coeffs[3] * t*t*t +    s_coeffs[4] * t*t*t*t + s_coeffs[5] * t*t*t*t*t;
            s_dot     =   s_coeffs[1] +      2*s_coeffs[2] * t +   3*s_coeffs[3] * t*t +   4*s_coeffs[4] * t*t*t +  5*s_coeffs[5] * t*t*t*t;
            s_dot_dot = 2*s_coeffs[2] * t +  6*s_coeffs[3] * t +  12*s_coeffs[4] * t*t +  20*s_coeffs[5] * t*t*t;
            
            d         =   d_coeffs[0] +        d_coeffs[1] * t +     d_coeffs[2] * t*t +     d_coeffs[3] * t*t*t +    d_coeffs[4] * t*t*t*t + d_coeffs[5] * t*t*t*t*t;
            d_dot     =   d_coeffs[1] +      2*d_coeffs[2] * t +   3*d_coeffs[3] * t*t +   4*d_coeffs[4] * t*t*t +  5*d_coeffs[5] * t*t*t*t;
            d_dot_dot = 2*d_coeffs[2] * t +  6*d_coeffs[3] * t +  12*d_coeffs[4] * t*t +  20*d_coeffs[5] * t*t*t;
                     
            path_s.push_back({s, s_dot, s_dot_dot});
            path_d.push_back({d, d_dot, d_dot_dot});
            previous_path_x.push_back(gran_map.spline_x(s) + d * gran_map.spline_dx(s));
            previous_path_y.push_back(gran_map.spline_y(s) + d * gran_map.spline_dy(s));   
          }

          
          //DEBUG
          for(int i = 1; i < path_s.size(); i++) {
            double s_dif = path_s[i][0] - path_s[i-1][0];
            std::cout << s_dif << std::endl;
            }
          std::cout << std::endl << std::endl;

          msgJson["next_x"] = previous_path_x;
          msgJson["next_y"] = previous_path_y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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