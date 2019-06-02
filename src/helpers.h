#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include "spline.h"
#include "Eigen-3.3/Eigen/Dense"
//#include "Eigen-3.3/Eigen/QR"


// for convenience
using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  //int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
  int next_wp = ClosestWaypoint(x,y, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};

  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
  }

  return result;
}

struct granular_map {
  vector<double> x;
  vector<double> y;
  vector<double> dx;
  vector<double> dy;
  vector<double> s;
  tk::spline spline_x;
  tk::spline spline_y;
  tk::spline spline_dx;
  tk::spline spline_dy;
};

granular_map get_map(tk::spline x, tk::spline y, tk::spline dx, tk::spline dy, double MAX_S) {
  
  vector<double> new_x;
  vector<double> new_y;
  vector<double> new_dx;
  vector<double> new_dy;
  vector<double> new_s;
  
  float incrementer = 0.02;
  for (double s = 0; s <= floor(MAX_S); s+=incrementer) {
    new_x.push_back(x(s));
    new_y.push_back(y(s));
    new_dx.push_back(dx(s));
    new_dy.push_back(dy(s));
    new_s.push_back(s);
  }
  
  granular_map result = {new_x, new_y, new_dx, new_dy, new_s, x, y, dx, dy};
  return result;
}

int getlane(double d) {
  int result = ceil(d/4);
  return result;
}

vector<vector<double>> find_cars_in_lane(vector<vector <double>> sensor_fusion, double lane) {
  vector<vector <double>> cars;
  for(vector<double> vehicle : sensor_fusion) { //find all cars in lane ahead of me
    if(getlane(vehicle[6]) == lane) {
      cars.push_back({vehicle[5], sqrt(vehicle[3]*vehicle[3] + vehicle[4]*vehicle[4])});
    }
  }
  return cars;
}
double get_s_distance(double follower, double leader, double max_s) {
  double distance = leader - follower;
  if (distance < -2000) {
    distance += max_s;
  } else if (distance > 2000) {
    distance -= max_s;
  }
  return distance;
}

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
    int num_points_required = coeffs[1][0]/timeframe;
    //std::cout << num_points_required << std::endl;
    vector<double> s_coeffs = coeffs[0];
    //s_coeffs[0] << ", " << s_coeffs[1] << ", " << s_coeffs[2] << ", " << s_coeffs[3] << ", " << s_coeffs[4] << ", " << s_coeffs[5] << std::endl;
    driveable = true;
    cost = 1000;
    for(int i = 1; i < num_points_required ; ++i) {
      t = i * timeframe;
      s         =   s_coeffs[0] +        s_coeffs[1] * t +     s_coeffs[2] * t*t +     s_coeffs[3] * t*t*t +    s_coeffs[4] * t*t*t*t + s_coeffs[5] * t*t*t*t*t;
      s_dot     =   s_coeffs[1] +      2*s_coeffs[2] * t +   3*s_coeffs[3] * t*t +   4*s_coeffs[4] * t*t*t +  5*s_coeffs[5] * t*t*t*t;
      s_dot_dot = 2*s_coeffs[2] *   +  6*s_coeffs[3] * t +  12*s_coeffs[4] * t*t +  20*s_coeffs[5] * t*t*t;
      s_ddd     = 6*s_coeffs[3]     + 24*s_coeffs[4] * t +  60*s_coeffs[5] * t*t;
      s = fmod(s, max_location);
	  
      trajectory.push_back({s, s_dot, s_dot_dot});
      if ((s_dot < min_speed) or (s_dot > max_speed) or (abs(s_dot_dot) >= max_accel) or (abs(s_ddd) >= max_jerk) or (d_trajectory == true and ((s < 1) or (s > 11)))) {
        driveable = false;
        //std::cout << i << "{" << s << ", " << s_dot << ", " << s_dot_dot << ", " << s_ddd << "}" << std::endl; 
        break;
      } 
    }
    if (driveable == true) { 
       if (d_trajectory == false){
         //cost = 1/(get_s_distance(trajectory.front()[0], trajectory.back()[0], max_location)/trajectory.size());
         cost = abs(trajectory.back()[2]);
       } else {
         double size = trajectory.size();
         cost = 1/size;         
       }
    }
    if (driveable = true and cost < min_cost and cost > 0) {
      result = trajectory;
      double size = result.size();
      //std::cout << min_cost << std::endl;
      min_cost = cost;
      //std::cout << "Driveable = " << driveable << " Cost = " << cost << " d: " << d_trajectory << "  (" << trajectory.front()[0] << ", " << trajectory.back()[0] << ") with points: " << size << " and pathtime: " << coeffs[1][0] << std::endl;
    }
    trajectory = {};
   }
  return result;
}

   
      
      
#endif  // HELPERS_H