#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "trajectory.h"
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
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s, &map_waypoints_dx,&map_waypoints_dy, &gran_map, &path_s, &path_d, &max_s]
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
          vector<double> x_path = previous_path_x;
          vector<double> y_path = previous_path_y;
          
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
          double speed_limit = 49 * 0.44704;
         
          //begin trajectory creation
          int state;
          double pathtime;
          bool recalc = true;
          bool change_behavior = false;
          int vary_d = 0;
          int elapsed_points;
          int intended_lane;
          double target_velocity;
          double target_position;
          double closest_obstacle;
          int my_lane = getlane(car_d);
          double buff;  //variable to determine how much of the previous path to include in the subsquent path
          int num_points_required;// = pathtime/timeframe - buff;
          int transition_points;// = num_points_required - elapsed_points;
          double time_required;// = num_points_required*timeframe;
          vector<double> s_start;
          vector<double> d_start;
          vector<double> s_end;
          vector<double> d_end;
          vector<vector<vector <double>>> s_coeffs_package;
          vector<vector<vector <double>>> d_coeffs_package;
          vector<vector <double>> s_trajectory_package;
          vector<vector <double>> d_trajectory_package;
          vector<vector <double>> s_trajectory;
          vector<vector <double>> d_trajectory;
          int j_min, j_max, i_min, i_max;
          
          //initialization
          if (previous_path_y.size() == 0 ) {  //initialization when there is no previous path
            path_s.push_back({car_s, car_speed * 0.44704, 0});
            path_d.push_back({car_d, 0, 0});
            previous_path_x.push_back(gran_map.spline_x(car_s) + car_d * gran_map.spline_dx(car_s));
            previous_path_y.push_back(gran_map.spline_y(car_s) + car_d * gran_map.spline_dy(car_s));
            x_path.push_back(gran_map.spline_x(car_s) + car_d * gran_map.spline_dx(car_s));
            y_path.push_back(gran_map.spline_y(car_s) + car_d * gran_map.spline_dy(car_s));
            buff=0;
          } else { //initialization when a previous path exists
 			//remove points that have been used from the s and d paths
            elapsed_points = path_s.size() - previous_path_y.size();
            path_s.erase(path_s.begin(),path_s.begin()+elapsed_points);
            path_d.erase(path_d.begin(),path_d.begin()+elapsed_points);
            buff=0;
          }
          
          
          // Behavior planning
          state = 1;

          
          // get trajectory sets based on state choice
          if (state == 1) { //state for stay in lane
            intended_lane = my_lane;     
            vector<double> closest_car = car_to_follow(sensor_fusion, car_s, car_d, max_s);
            vary_d = 0;
            double end_pos;
            if (closest_car[0] == 0) { //case where there is no car in the lane
              target_velocity = speed_limit - 2; 
              end_pos = path_s.back()[0] + elapsed_points*timeframe*target_velocity;
              closest_obstacle = 0;
            } else { // case where there is a car in the lane that we need to follow at a safe distance
              target_velocity = closest_car[2];
              closest_obstacle = get_s_distance(car_s, closest_car[1], max_s) - 3;
              end_pos = (path_s.back()[0] + closest_obstacle) + elapsed_points*timeframe*target_velocity;              
            }
            recalc = !(abs(path_s.back()[1] - target_velocity) < 1);
            std::cout << "recalc: " << recalc << std::endl;
            if (recalc == false) { //just tack a few new points onto the previous path since we don't need to recalculate
              s_start = {path_s.back()[0], path_s.back()[1], path_s.back()[2]};
              d_start = {path_d.back()[0], path_d.back()[1], path_d.back()[2]};
              s_end = {end_pos, target_velocity, 0};
              d_end = {double(intended_lane*4-2), 0, 0};
              vector<double> sco = JMT(s_start, s_end, elapsed_points*timeframe);
              s_coeffs_package.push_back({sco, {elapsed_points*timeframe}});
              vector<double> dco = JMT(d_start, d_end, elapsed_points*timeframe);
              d_coeffs_package.push_back({dco, {elapsed_points*timeframe}});
            } else {  //build a new path from scrath
              path_s = {path_s[0]};
              path_d = {path_d[0]};
              previous_path_x = {previous_path_x[0]};
              previous_path_y = {previous_path_y[0]};
              //create package of beginnig and end {s, sdot, sdotdot} vectors for JMT calculation by varying final position and total path time.
              calculate_coeff_package(s_coeffs_package, d_coeffs_package, path_s, path_d, 0, target_velocity, closest_obstacle, intended_lane);
            }
          }
          int scoefsz = s_coeffs_package.size();
          //std::cout << "Number of possible s polynomials: " << scoefsz << std::endl; 



          //create JMT polynomicals and rank them 
          s_trajectory = rank_trajectories(s_coeffs_package, max_s, speed_limit,         -1,    timeframe, false, 10, 20);
          d_trajectory = rank_trajectories(d_coeffs_package, 1000,  speed_limit,  -speed_limit, timeframe, true,  10, 20);
          int tmp_sz_s = s_trajectory.size();
          int tmp_sz_d = d_trajectory.size();
          double dist_traveled = s_trajectory.back()[0]-s_trajectory.front()[0];
          double avg_vel = (dist_traveled)/(tmp_sz_s*.02);
          //std::cout << "s trajectory size: " << tmp_sz_s << ",  distance traveled: " << dist_traveled << " and average speed: " << avg_vel << std::endl;
          //std::cout << "d trajectory size: " << tmp_sz_d << std::endl;

          //std::cout << "Trajectory (s,d)   (x,y):" << std::endl;
          if(s_trajectory.size() > 1 and d_trajectory.size() > 1) {
            for(int i = 1; i < s_trajectory.size(); i++) {
              path_s.push_back(s_trajectory[i]);
              if (d_trajectory.size()  <= i) {
                d_trajectory.push_back({d_trajectory.back()[0], 0, 0});
              }
              path_d.push_back(d_trajectory[i]);
              //std::cout << i << "    s: {" << s_trajectory[i][0] << ", " << s_trajectory[i][1] << ", " << s_trajectory[i][2] << "}    d: {" << d_trajectory[i][0] << ", " << d_trajectory[i][1] << ", " << d_trajectory[i][2] << "}" << std::endl;
              double s = s_trajectory[i][0];
              double d = d_trajectory[i][0];
              double x = gran_map.spline_x(s) + d * gran_map.spline_dx(s);
              double y = gran_map.spline_y(s) + d * gran_map.spline_dy(s);
              previous_path_x.push_back(x);
              previous_path_y.push_back(y); 
              x_path.push_back(x);
              y_path.push_back(y);
            }
          }

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