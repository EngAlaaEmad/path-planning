#ifndef PLANNER_H
#define PLANNER_H

#include "vehicle.h"

const double TIME_STEP = 0.02;
const double MPH_TO_MPS = 0.447;
const double MPS_TO_MPH = 2.24;

struct Map{
  vector<double> waypoints_x;
  vector<double> waypoints_y;
  vector<double> waypoints_s;
  vector<double> waypoints_dx;
  vector<double> waypoints_dy;
};

class Planner {

    private:

        double max_speed;
        double max_acceleration;
        double ref_speed;
        double lane_width;
        double following_time;
        double min_time_in_lane;

    public:

        void initialize(Vehicle &car, int lane, int lane_width, double max_speed, double start_speed, double max_acceleration, double following_time, double min_time_in_lane);
        vector<string> get_successor_states(Vehicle car);
        void set_speed(Vehicle &car, vector<vector<double>> sensor_data);
        vector<vector<double>> generate_trajectory(Vehicle &car, vector<double> previous_path_x, vector<double> previous_path_y, double prev_path_end_s, Map &road_map);
        
        double lane_change_cost(string state, Vehicle car, vector<vector<double>> sensor_data); 
        double lane_speed_cost(string state, Vehicle car, vector<vector<double>> sensor_data);
        double num_of_vehicles_cost(string state, Vehicle car, vector<vector<double>> sensor_data);

};

#endif