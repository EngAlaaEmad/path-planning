#include "helpers.h"
#include "planner.h"
#include "vehicle.h"
#include "iostream"

Planner::Planner() {}

Planner::~Planner() {}

vector<vector<double>> Planner::generate_trajectory(string state, Vehicle &car, vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
{
    int lane = 1;
    if (state == "KEEP_LANE")
    {
        lane = car.lane;
        std::cout << "keeping in lane " << lane << std::endl;
    }
    else if (state == "LANE_CHANGE_LEFT"){
        lane = car.lane - 1;
        std::cout << "changing to lane " << lane << std::endl;
    }
    else if (state == "LANE_CHANGE_RIGHT"){
        lane = car.lane + 1;
        std::cout << "changing to lane " << lane << std::endl;
    }

    

    vector<double> anchor_x;
    vector<double> anchor_y;

    double ref_x = car.x;
    double ref_y = car.y;
    double ref_yaw = deg2rad(car.yaw);

    int prev_size = previous_path_x.size();

    if (prev_size < 2)
    {

        // Use two points to make the path tangent to the car
        double prev_car_x = car.x - cos(car.yaw);
        double prev_car_y = car.y - sin(car.yaw);

        anchor_x.push_back(prev_car_x);
        anchor_x.push_back(car.x);

        anchor_y.push_back(prev_car_y);
        anchor_y.push_back(car.y);
    }

    else
    {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        anchor_x.push_back(ref_x_prev);
        anchor_x.push_back(ref_x);

        anchor_y.push_back(ref_y_prev);
        anchor_y.push_back(ref_y);
    }

    // Add 30m evenly spaced points converted from Frenet coordinates to XY
    vector<double> next_wp0 = getXY(car.s + 30, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car.s + 60, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car.s + 90, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    anchor_x.push_back(next_wp0[0]);
    anchor_x.push_back(next_wp1[0]);
    anchor_x.push_back(next_wp2[0]);

    anchor_y.push_back(next_wp0[1]);
    anchor_y.push_back(next_wp1[1]);
    anchor_y.push_back(next_wp2[1]);

    // Transform to have car or end of previous path as origin
    for (int i = 0; i < anchor_x.size(); i++)
    {

        double shift_x = anchor_x[i] - ref_x;
        double shift_y = anchor_y[i] - ref_y;

        anchor_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        anchor_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // Create spline from anchor points
    tk::spline s;
    s.set_points(anchor_x, anchor_y);

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // Start with points remaining from previous path
    for (int i = 0; i < prev_size; i++)
    {

        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Calculate step size of spline for desired speed
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double x_add_on = 0;

    // Fill up rest of path with interpolated values
    for (int i = 1; i <= 50 - prev_size; i++)
    {

        double N = target_dist / (0.02 * car.desired_speed / 2.24);
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // Rotate back to normal
        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    vector<vector<double>> trajectory;
    trajectory.push_back(next_x_vals);
    trajectory.push_back(next_y_vals);

    car.lane = lane;

    return trajectory;
}

vector<string> Planner::get_successor_states(Vehicle car)
{

    vector<string> states;
    states.push_back("KEEP_LANE");

    string state = car.state;

    if (state == "KEEP_LANE")
    {
        if (car.lane != 0){
            states.push_back("LANE_CHANGE_LEFT");
        }
        if (car.lane != 2)
        {
           states.push_back("LANE_CHANGE_RIGHT"); 
        }
        
        
    }
    // If state is "LCL" or "LCR", then just return "KL"
    return states;
}

int Planner::lane_change_cost(string state, Vehicle car, vector<vector<double>> sensor_data){

    int desired_lane = 1;
    
    if (state == "KEEP_LANE")
    {
        return 0;
    }
    else if (state == "LANE_CHANGE_LEFT"){
        desired_lane = car.lane - 1;
    }
    else if (state == "LANE_CHANGE_RIGHT"){
        desired_lane = car.lane + 1;
    }

    int lane_change_cost = 2;

    for (int i = 0; i < sensor_data.size(); i++) {
        // data for ith car
        float d = sensor_data[i][6];

        // check if car is in our lane
        if (d < (2 + 4 * desired_lane + 2) && d > (2 + 4 * desired_lane - 2)){

            double vx = sensor_data[i][3];
            double vy = sensor_data[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            double check_car_s = sensor_data[i][5];

            if (abs(check_car_s - car.s) < 10){
                std::cout << "car in lane " << desired_lane << std::endl;
                lane_change_cost = 99999;
                break;
            }
            
        }
    }
    //std::cout << "lane change cost: " << lane_change_cost << std::endl;

    return lane_change_cost;

}

double Planner::lane_speed_cost(string state, Vehicle car, vector<vector<double>> sensor_data){

    int desired_lane = 1;
    if (state == "KEEP_LANE")
    {
        desired_lane = car.lane;
    }
    else if (state == "LANE_CHANGE_LEFT"){
        desired_lane = car.lane - 1;
    }
    else if (state == "LANE_CHANGE_RIGHT"){
        desired_lane = car.lane + 1;
    }

    double average_speed = 0.0;
    int num_of_relevant_cars = 0;

    for (int i = 0; i < sensor_data.size(); i++) {
        // data for ith car
        float d = sensor_data[i][6];
        double check_car_s = sensor_data[i][5];

        // check if car is in our lane and ahead of us but not too far
        if (d < (2 + 4 * desired_lane + 2) && d > (2 + 4 * desired_lane - 2) && check_car_s > car.s && check_car_s - car.s < 60){

            double vx = sensor_data[i][3];
            double vy = sensor_data[i][4];
            double check_speed = sqrt(vx * vx + vy * vy);
            average_speed += check_speed;
            num_of_relevant_cars++;
            
        }
    }

    average_speed = (num_of_relevant_cars > 0) ? (average_speed * 2.24 / num_of_relevant_cars) : (49.5);
    double lane_speed_cost = 49.5 - average_speed;

    return lane_speed_cost;
}

double Planner::num_of_vehicles_cost(string state, Vehicle car, vector<vector<double>> sensor_data){
    
    int desired_lane = 1;
    if (state == "KEEP_LANE")
    {
        desired_lane = car.lane;
    }
    else if (state == "LANE_CHANGE_LEFT"){
        desired_lane = car.lane - 1;
    }
    else if (state == "LANE_CHANGE_RIGHT"){
        desired_lane = car.lane + 1;
    }

    double num_of_vehicles = 0.0;

        for (int i = 0; i < sensor_data.size(); i++) {
            // data for ith car
            float d = sensor_data[i][6];

            // check if car is in our lane
            if (d < (2 + 4 * desired_lane + 2) && d > (2 + 4 * desired_lane - 2)){
                num_of_vehicles++;       
            }
        }

    return num_of_vehicles;

}