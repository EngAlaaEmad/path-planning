#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "vehicle.h"

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

  // Set lane and speed
  int lane = 1;
  double current_speed = 0.0; // mph

  const double MAX_SPEED = 49.5;
  double ref_speed = MAX_SPEED;

  h.onMessage([&lane, &current_speed, &ref_speed, &MAX_SPEED, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          double x = j[1]["x"];
          double y = j[1]["y"];
          double s = j[1]["s"];
          double d = j[1]["d"];
          double yaw = j[1]["yaw"];
          double speed = j[1]["speed"];
          Vehicle car(x, y, s, d, yaw, speed);

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Number of points remaining from previous path
          int prev_size = previous_path_x.size();

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          if (prev_size > 0){
            car.s = end_path_s;
          }

          bool car_ahead = false;

          for (int i = 0; i < sensor_fusion.size(); i++){
            // data for ith car
            float d = sensor_fusion[i][6];

            // check if car is in our lane
            if (d < (2+4*lane+2) && d > (2+4*lane-2)){
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];

              // project cars s value out to end of path (future pos)
              check_car_s += (double)prev_size * 0.02 * check_speed;

              if ((check_car_s > car.s) && (check_car_s - car.s < 2*current_speed*0.447)){
                car_ahead = true;
                ref_speed = check_speed;
                std::cout << "Car ahead, set refspeed to " << ref_speed << std::endl;

              }

            }
          }

          std::cout << "Car ahead: " << car_ahead << std::endl;
          
          // Slow down if too close
          if (car_ahead == true && current_speed > ref_speed){
            std::cout << "Slowing down..." << std::endl;
            current_speed -= 0.224;
          }
          // Otherwise speed up incrementally
          else if (current_speed < MAX_SPEED){
            current_speed += 0.224;
            std::cout << "Accelerating..." << std::endl;
          }

          vector<vector<double>> trajectory = generate_trajectory(car, current_speed, lane, previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_x_vals = trajectory[0];
          vector<double> next_y_vals = trajectory[1];
          
          // END#include "spline.h"
          json msgJson;

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