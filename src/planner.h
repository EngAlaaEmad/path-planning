#ifndef PLANNER_H
#define PLANNER_H

#include "vehicle.h"

class Planner {

    public:
        vector<vector<double>> generate_trajectory(Vehicle car, double lane, vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y);



};

#endif