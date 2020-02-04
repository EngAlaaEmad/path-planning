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
  const double MAX_SPEED = 49.5;
  double ref_speed = MAX_SPEED;

  Vehicle car;
  Planner path_planner;

  h.onMessage([&lane, &ref_speed, &MAX_SPEED, &car, &path_planner, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
          car.x = j[1]["x"];
          car.y = j[1]["y"];
          car.s = j[1]["s"];
          car.d = j[1]["d"];
          car.yaw = j[1]["yaw"];
          car.speed = j[1]["speed"];

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


          vector<string> possible_states = path_planner.get_successor_states(car);
          vector<double> costs;

          /*std::cout << "Possible states: ";
          for (int i = 0; i < possible_states.size(); i++){
            std::cout << possible_states[i] << " ";
          }
          std::cout << std::endl;*/

          for (int i = 0; i < possible_states.size(); i++){
            double cost_for_state = 0.0;
            cost_for_state += path_planner.lane_change_cost(possible_states[i], car, sensor_fusion);
            cost_for_state += path_planner.lane_speed_cost(possible_states[i], car, sensor_fusion);
            cost_for_state += path_planner.num_of_vehicles_cost(possible_states[i], car, sensor_fusion);
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

          /*std::cout << "Costs: ";
          for (int i = 0; i < costs.size(); i++){
            std::cout << costs[i] << " ";
          }
          std::cout << std::endl;

          std::cout << "Next state: " << best_state << std::endl;*/

          car.set_speed(ref_speed, MAX_SPEED, previous_path_x, previous_path_y, end_path_s, sensor_fusion);

          vector<vector<double>> trajectory = path_planner.generate_trajectory(best_state, car, previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_x_vals = trajectory[0];
          vector<double> next_y_vals = trajectory[1];

          

          // std::cout << "Vehicle state: " << car.state << std::endl;
          
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