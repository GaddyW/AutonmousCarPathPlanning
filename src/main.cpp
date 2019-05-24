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
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s, &map_waypoints_dx,&map_waypoints_dy, &gran_map]
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

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          vector<double> next_s_vals;
          vector<double> next_d_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          double timeframe = 0.02;
          double pathtime = 2.0;
          double target_velocity = 10.0;
          double t;
          
		  //get coeffecients
          vector<double> s_coeffs;
          vector<double> d_coeffs;
          vector<double> next_xy;
          double next_s;
          double next_d;
          int num_points_required;
          double time_required;
          double start_s;
          double start_speed;
          vector<double> s_start;
    	  vector<double> s_end;
          vector<double> d_start;
    	  vector<double> d_end;
          
          
          
          int prev_path_size = previous_path_y.size();
          /*if(prev_path_size > 0){
            vector<double> calcxy = getXY(end_path_s, end_path_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            double dist = distance(calcxy[0], calcxy[1], previous_path_x.back(), previous_path_y.back());
            std::cout << "Distance: " << dist << "  S/D: (" << calcxy[0] << "," << calcxy[1] << ")"  << "  XY paths: (" << previous_path_x.back() << "," << previous_path_y.back() << ")  " << end_path_d << std::endl;
            
          }*/
          double buff = prev_path_size - .5/timeframe;
          if (buff < 0) {
            buff = prev_path_size;
          } else {
            buff = .5/timeframe;
          }
          num_points_required = pathtime/timeframe - prev_path_size;
          time_required = num_points_required*timeframe;
          if (prev_path_size == 0 ) {
            start_s = car_s;
            start_speed = car_speed * 0.44704; //mph to m/s conversion
          } else {
            start_s = end_path_s + target_velocity*timeframe;
            start_speed = target_velocity; //vel;
          }
          s_start = {start_s, start_speed, 0};
          s_end = {start_s + target_velocity*time_required, target_velocity, 0};
          d_start = {6, 0, 0};
          d_end = {6, 0, 0};
          //std::cout << "Current X: " << car_x << std::endl;
          //std::cout << "Current Y: " << car_y << std::endl;
          //std::cout << "Current Yaw: " << car_yaw << std::endl;
          //std::cout << "Current S: " << car_s << std::endl;
          //std::cout << "Current Speed: " << car_speed << std::endl;
          //std::cout << "Prev Path Size: " << prev_path_size << std::endl;
          //std::cout << "Time to add on: " << time_required << std::endl;
          //std::cout << "S_start: " << s_start[0] << ", " << s_start[1] << ", " << s_start[2] << std::endl;
          //std::cout << "S_end: "   << s_end[0]  << ", " << s_end[1]  << ", " << s_end[2]  << std::endl;
          //std::cout << "D_start: " << d_start[0] << ", " << d_start[1] << ", " << d_start[2] << std::endl;
          //std::cout << "D_end: "   << d_end[0]  << ", " << d_end[1]  << ", " << d_end[2]  << std::endl;
          //std::cout << "Number of new points required:  " << num_points_required << std::endl;          
          
          
          s_coeffs = JMT(s_start, s_end, time_required);
          d_coeffs = JMT(d_start, d_end, time_required);
		  
          
          for(int i = 0; i < prev_path_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
                                  
          for(int i = 0; i < num_points_required ; ++i) {
    		t = i * timeframe;
            next_s = s_coeffs[0] + s_coeffs[1] * t + s_coeffs[2] * t*t + s_coeffs[3] * t*t*t + s_coeffs[4] * t*t*t*t + s_coeffs[5] * t*t*t*t*t;
            next_d = d_coeffs[0] + d_coeffs[1] * t + d_coeffs[2] * t*t + d_coeffs[3] * t*t*t + d_coeffs[4] * t*t*t*t + d_coeffs[5] * t*t*t*t*t;
            //next_xy = getXY(next_s, next_d, gran_map.s, gran_map.x, gran_map.y);
            next_s_vals.push_back(next_s);
            next_d_vals.push_back(next_d);
            //next_x_vals.push_back(next_xy[0]);
            //next_y_vals.push_back(next_xy[1]);
            next_x_vals.push_back(gran_map.spline_x(next_s) + next_d * gran_map.spline_dx(next_s));
            next_y_vals.push_back(gran_map.spline_y(next_s) + next_d * gran_map.spline_dy(next_s));           
          }
          //DEBUG
           std::cout << num_points_required << std::endl;
          for(int i = 1; i < next_x_vals.size(); i++) {
              double xy_dist = distance(next_x_vals[i], next_y_vals[i], next_x_vals[i-1], next_y_vals[i-1]);
              //double s_dist = next_s_vals[i] - next_s_vals[i-1];
              double xy_speed = xy_dist/timeframe;
              //double s_speed = s_dist/timeframe;
			  std::cout << "XY Distance: " << xy_dist << "  XY Speed: " << xy_speed << std::endl;
         	  //std::cout << "S Distance: " << s_dist << "  S Speed: " << s_speed << "   Car Speed: " << car_speed << "   XY Distance: "  << xy_dist << "   XY Speed: " << xy_speed << std::endl;

            }
		
          
      	        
            
          std::cout << std::endl << std::endl;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

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