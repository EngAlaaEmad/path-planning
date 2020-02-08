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
#include "planner.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  Map road_map;
  /*vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;*/

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
    road_map.waypoints_x.push_back(x);
    road_map.waypoints_y.push_back(y);
    road_map.waypoints_s.push_back(s);
    road_map.waypoints_dx.push_back(d_x);
    road_map.waypoints_dy.push_back(d_y);
  }

  int starting_lane = 1;
  int lane_width = 4; // m
  double max_speed = 49.5; // mph
  double start_speed = 0.0; // mph
  double max_acceleration = 0.448; // mph2
  double following_time = 1.75; // s
  double min_time_in_lane = 1.0; // s

  Vehicle car;
  Planner path_planner;
  path_planner.initialize(car, starting_lane, lane_width, max_speed, start_speed, max_acceleration, following_time, min_time_in_lane);

  h.onMessage([&car, &path_planner, &road_map]
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
          car.x = j[1]["x"];
          car.y = j[1]["y"];
          car.s = j[1]["s"];
          car.d = j[1]["d"];
          car.yaw = j[1]["yaw"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Previous path's end s and d values 
          double prev_path_end_s = j[1]["end_path_s"];
          double prev_path_end_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_data = j[1]["sensor_fusion"];

          // Get a list of possible FSM states based on current state
          vector<string> possible_states = path_planner.get_successor_states(car);
          
          // Calculate costs for each possible state
          vector<double> costs;

          for (int i = 0; i < possible_states.size(); i++){
            double cost_for_state = 0.0;
            cost_for_state += path_planner.lane_change_cost(possible_states[i], car, sensor_data);
            cost_for_state += path_planner.lane_speed_cost(possible_states[i], car, sensor_data);
            cost_for_state += path_planner.num_of_vehicles_cost(possible_states[i], car, sensor_data);
            costs.push_back(cost_for_state);
          }

          string best_state;
          double min_cost = 999999;

          for (int i = 0; i < costs.size(); i++){
            if (costs[i] < min_cost){
              min_cost = costs[i];
              best_state = possible_states[i];
            }
          }
          car.state = best_state;

          if (car.state == "KEEP_LANE"){
            car.keep_lane_cnt++;
          }
          else{
            car.keep_lane_cnt = 0;
          }

          // Set speed, either accelerating towards max_speed or following vehicle
          path_planner.set_speed(car, sensor_data);

          // Generate trajectory (x,y points) based on best next state
          vector<vector<double>> trajectory = path_planner.generate_trajectory(car, previous_path_x, previous_path_y, prev_path_end_s, road_map);
          vector<double> next_x_vals = trajectory[0];
          vector<double> next_y_vals = trajectory[1];
          
          // END
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