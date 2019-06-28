#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include "spline.h"
#include "Eigen-3.3/Eigen/Dense"
//#include "Eigen-3.3/Eigen/QR"


vector<double> car_to_follow(vector<vector <double>> sensor_fusion, double my_s, double my_d, double max_s) {
  vector<double> result;
  vector<vector <double>> cars = find_cars_in_lane(sensor_fusion, getlane(my_d));
  double lowest_distance = 1000000;
  
  result = {0, -1,-1};
  if (cars.size() > 0) {
    for(vector<double> vehicle : cars) {
      double distance = get_s_distance(my_s, vehicle[0],  max_s);
      if((distance > 0) and (distance < 50) and (distance < lowest_distance)) {
        result = {1, vehicle[0], vehicle[1]};
        lowest_distance = distance;
       // std::cout << "Car " << lowest_distance << " meters ahead at s=" << vehicle[0] << " traveling " << vehicle[1] << " m/s" << std::endl;
      }
    }
  }
  return result;
}


vector<vector<double>>  rank_trajectories(vector<vector<vector<double>>> coeffs_package, double max_location, double max_speed, double min_speed, double timeframe, bool d_trajectory, double max_accel, double max_jerk) {
  double s, s_dot, s_dot_dot, s_ddd;
  double t;
  double min_cost = 1000000;
  double cost = min_cost;
  bool driveable;
  vector<vector<double>> result;
  vector<vector<double>> trajectory = {};
  for(vector<vector<double>> coeffs : coeffs_package) {
    int num_points_required = std::max(coeffs[1][0]/timeframe,3.0);
    vector<double> s_coeffs = coeffs[0];
    driveable = true;
    cost = 1000;
    for(int i = 0; i < num_points_required ; ++i) {
      t = i * timeframe;
      s         =   s_coeffs[0] +        s_coeffs[1] * t +     s_coeffs[2] * t*t +     s_coeffs[3] * t*t*t +    s_coeffs[4] * t*t*t*t + s_coeffs[5] * t*t*t*t*t;
      s_dot     =   s_coeffs[1] +      2*s_coeffs[2] * t +   3*s_coeffs[3] * t*t +   4*s_coeffs[4] * t*t*t +  5*s_coeffs[5] * t*t*t*t;
      s_dot_dot = 2*s_coeffs[2] +      6*s_coeffs[3] * t +  12*s_coeffs[4] * t*t +  20*s_coeffs[5] * t*t*t;
      s_ddd     = 6*s_coeffs[3] +     24*s_coeffs[4] * t +  60*s_coeffs[5] * t*t;
      s = fmod(s, max_location);
	  
      trajectory.push_back({s, s_dot, s_dot_dot});
      if ((s_dot < min_speed) or (s_dot > max_speed) or (abs(s_dot_dot) >= max_accel) or (abs(s_ddd) >= max_jerk) or (d_trajectory == true and ((s < 1) or (s > 11)))) {
        driveable = false;
        break;
      } 
    }
    if (driveable == true) { 
      if (d_trajectory == false){
        cost = 1/(get_s_distance(trajectory.front()[0], trajectory.back()[0], max_location)/trajectory.size());
      } else {
        double size = trajectory.size();
        cost = 1/size;         
      }
    }
    if (driveable = true and cost < min_cost and cost > 0) {
      result = trajectory;
      double size = result.size();
      min_cost = cost;
    }
    trajectory = {};
   }
  return result;
}


void calculate_coeff_package(vector<vector<vector <double>>> &s_coeffs_package, vector<vector<vector <double>>> &d_coeffs_package, 
                             vector<vector <double>> &path_s, vector<vector <double>> &path_d, double buff, double speed, double safety_distance, int intended_lane ) {
  for(int j = 2; j < 30; j++) {
    double pathtime = j*0.4; //build set of potential path times that increment every 0.4 seconds
    //double max_position = std::min((speed_limit * pathtime)*2, (closest_obstacle - 3 + pathtime*speed_limit)*2);
    double max_position = (safety_distance+speed*pathtime)*2;
    for(int i = 0; i < max_position; i++) { //cap position increment at closest obstacle or at speed_limit * pathtime
      double pos = path_s[buff][0] + i/2; //build set of potential path times that increment by a meter every time
      double dend = double(intended_lane*4-2) ;                
      vector<double> s_start = {path_s[buff][0], path_s[buff][1], path_s[buff][2]};
      vector<double> d_start = {path_d[buff][0], path_d[buff][1], path_d[buff][2]};
      vector<double> s_end = {pos, speed, 0};
      vector<double> d_end = {dend, 0, 0};
      //std::cout << "S start: " << "{" << path_s[buff][0] << ", " << path_s[buff][1] << ", " << path_s[buff][2] << "}" << std::endl;
      //std::cout << "S end: " << "{" << pos << ", " << target_velocity << ", " << 0 << "}" << std::endl;
      vector<double> sco = JMT(s_start, s_end, pathtime);
      s_coeffs_package.push_back({sco, {pathtime}});
      vector<double> dco = JMT(d_start, d_end, pathtime);
      d_coeffs_package.push_back({dco, {pathtime}});
    }
  }
}

#endif  // TRAJECTORY_H