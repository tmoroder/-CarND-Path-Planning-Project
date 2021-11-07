#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"


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


  // lane and velocity variables
  int lane = 1;
  double ref_vel = 0.0;


  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane]
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

          // ===================
          // INSERTED CODE BEGIN
          // ===================


          // Additional variables
          int prev_size = previous_path_x.size();



          // Behavior:
          // Effectively determines the lane and velocity variables. This is done by first checking the near field of each lane 
          // for cars in the future, and then determining a lane change and/or velocity change.

          // Setting
          double front_distance = 30.0;
          double back_distance = 20.0;
          double max_speed = 49.5;

          // Predict lane status in front and in back field of view:
          // Compare the s value at the previous paths end (or current position if empty) to the 
          // one of each other car assuming a constant velocity
          vector<bool> is_lane_free_front {true, true, true};
          vector<bool> is_lane_free_back {true, true, true};
          double ref_s = car_s;
          if (prev_size > 0) {
            ref_s = end_path_s;
          }
          for (int i = 0; i < sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double speed = distance(vx, vy, 0, 0);
            double check_car_s = sensor_fusion[i][5];
            check_car_s += ((double) prev_size * 0.02 * speed);
            for (int j = 0; j < 3; ++j) {
              int lane_j = j;
              if ((2.+4.*lane_j-2.) < d && d < (2.+4.*lane_j+2.)) {
                if ((check_car_s > ref_s) && (check_car_s - ref_s) < front_distance) {
                  is_lane_free_front[j] = false;
                }
                if ((check_car_s < ref_s) && (ref_s - check_car_s) < back_distance) {
                  is_lane_free_back[j] = false;
                }
              }
            }
          
          }

          // Logic, in verbal terms:
          // - If current lane is blocked, then do left or right lane change, otherwise break.
          // - If lane is free then optionally increase speed if and optionally change to middle lane again.
          bool car_in_front = !is_lane_free_front[lane];
          if (car_in_front) {
            // Car in front is too close
            if ((lane > 0) && is_lane_free_front[lane - 1] && is_lane_free_back[lane - 1])  {
              // Left lane change
              lane = lane - 1;              
            } else if ((lane < 2) && is_lane_free_front[lane + 1] && is_lane_free_back[lane + 1]) {
              // Right lane change
              lane = lane + 1;
            } else {
              // Reduce speed
              ref_vel -= 0.224;
            }
          } else {
            // No car in front
            if (ref_vel < max_speed) {
              // Increase speed if not max yet
              ref_vel += 0.224;
            } 
            if (lane != 1) {
              // Change to middle lane if possible
              if ((lane > 0) && is_lane_free_front[lane - 1] && is_lane_free_back[lane - 1])  {
                lane = lane - 1;              
              } else if ((lane < 2) && is_lane_free_front[lane + 1] && is_lane_free_back[lane + 1]) {
                lane = lane + 1;
              }
            }
          }



      
          // The employed SPLINE fitting procedure requires an external provided set anchor points or knots.
          // We are using 5 points here:
          // - The last two points from the previous path (or computed form the initial heading)
          // - 3 planned targets for chosen lane at 30, 60 and 90 meters

          vector<double> ptsx;
          vector<double> ptsy;                 
          // Need coordinates/heading of the second anchor (stored in ref_) for a transformation later.
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          ref_s = car_s;
          
          // Previous 2 points
          if (prev_size < 2) {
            // Deduce "previous" point from heading
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // Pick last two points from previous path
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            ref_s = end_path_s;
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // Add 3 planned future points in 30, 60 and 90 units apart for current lane variable.
          vector<double> next_wp0 = getXY(ref_s + 30.0, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(ref_s + 60.0, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(ref_s + 90.0, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          

          // Transform into coordinates local to second anchor point and orientation pointing into +x axis. 
          // Otherwise SPLINE fit y=y(x) is not a function but a curve (at one given x we have multiple y values).
          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }


          // Performe SPLINE fitting
          tk::spline s;
          s.set_points(ptsx, ptsy);


          // Build next path, which consists of two components:
          // - Previous path points
          // - Additional new smoothed points

          // i) Previous points
          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // ii) Append additional points until the path has 50 points in total
          // Compute distance and divide lenth into equidistant parts consistent with velocity.
          // Then this number is used to regularly space the x coordinates at which respective y values are 
          // computed, transformed and appended. This is x coordinate spacing is an approximation and might 
          // change the overall velocity but it is a sufficient approximation here (and we 
          // usually only compute 1 additional point via this).
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = distance(target_x, target_y, 0.0, 0.0);
          double x_add_on = 0;
          double N = (target_dist / (0.02 * ref_vel / 2.24));
          for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);
            x_add_on = x_point;
            // inverse transformation
            double x_ref = x_point;
            double y_ref = y_point;
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            x_point += ref_x;
            y_point += ref_y;
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }



          // =================
          // INSERTED CODE END
          // =================


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