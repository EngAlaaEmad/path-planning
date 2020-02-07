#include "helpers.h"
#include "planner.h"
#include "vehicle.h"
#include "iostream"

void Planner::initialize(Vehicle &car, int lane, int lane_width, double max_speed, double start_speed, double max_acceleration, double following_time, double min_time_in_lane){
    this->max_speed = max_speed;
    this->lane_width = lane_width;
    this->max_acceleration = max_acceleration;
    this->following_time = following_time;
    this->min_time_in_lane = min_time_in_lane;
    car.lane = lane;
    car.current_speed = start_speed;
}

vector<string> Planner::get_successor_states(Vehicle car)
{

    vector<string> states;
    states.push_back("KEEP_LANE");

    // Only allow lane changes after keeping lane for at least 1 second
    if (car.state == "KEEP_LANE" && car.keep_lane_cnt > (min_time_in_lane / TIME_STEP))
    {
        if (car.lane != 0){
            states.push_back("LANE_CHANGE_LEFT");
        }
        if (car.lane != 2)
        {
           states.push_back("LANE_CHANGE_RIGHT"); 
        }
        
    }
    // If state is "LANE_CHANGE_LEFT" or "LANE_CHANGE_RIGHT", then just return "KEEP_LANE"
    return states;
}

void Planner::set_speed(Vehicle &car, vector<vector<double>> sensor_data){

  double leading_vehicle_speed = 0.0;
  bool car_ahead = false;
  double lane_edge_left = lane_width * car.lane;
  double lane_edge_right = lane_width * (car.lane + 1);

  for (int i = 0; i < sensor_data.size(); i++)
  {
    // d coordinate for ith car
    float vehicle_d = sensor_data[i][6];

    // check if car is in our lane
    if (vehicle_d > lane_edge_left && vehicle_d < lane_edge_right)
    {
      double vx = sensor_data[i][3];
      double vy = sensor_data[i][4];
      double vehicle_speed = sqrt(vx * vx + vy * vy);
      double vehicle_s = sensor_data[i][5];

      // check for vehicles ahead of us, keeping distance equivalent to 1.5 seconds
      if ((vehicle_s > car.s) && (vehicle_s - car.s < following_time * (car.current_speed) * MPH_TO_MPS))
      {
        car_ahead = true;
        leading_vehicle_speed = vehicle_speed;
      }
    }
  }

  // Slow down if too close
  if (car_ahead && car.current_speed > leading_vehicle_speed)
  {
    car.current_speed -= max_acceleration;
  }

  // Otherwise speed up incrementally
  else if (car.current_speed < max_speed)
  {
    car.current_speed += max_acceleration;
  }

}

vector<vector<double>> Planner::generate_trajectory(Vehicle &car, vector<double> previous_path_x, vector<double> previous_path_y, double prev_path_end_s, Map &road_map)
{
    if (car.state == "KEEP_LANE")
    {
        std::cout << "keeping in lane " << car.lane << std::endl;
    }
    else if (car.state == "LANE_CHANGE_LEFT"){
        car.lane -= 1;
        std::cout << "changing to lane " << car.lane << std::endl;
    }
    else if (car.state == "LANE_CHANGE_RIGHT"){
        car.lane += 1;
        std::cout << "changing to lane " << car.lane << std::endl;
    }

    vector<double> anchor_pts_x;
    vector<double> anchor_pts_y;

    double ref_x = car.x;
    double ref_y = car.y;
    double ref_yaw = deg2rad(car.yaw);

    int num_of_remaining_points = previous_path_x.size();

    // Start new trajectory from end of the previous one
    double trajectory_starting_s = car.s;
    if (num_of_remaining_points > 0){
        trajectory_starting_s = prev_path_end_s;
    }

    if (num_of_remaining_points < 2)
    {
        // Use two points to make the path tangent to the car
        double prev_car_x = car.x - cos(car.yaw);
        double prev_car_y = car.y - sin(car.yaw);

        anchor_pts_x.push_back(prev_car_x);
        anchor_pts_x.push_back(car.x);

        anchor_pts_y.push_back(prev_car_y);
        anchor_pts_y.push_back(car.y);
    }

    else
    {
        ref_x = previous_path_x[num_of_remaining_points - 1];
        ref_y = previous_path_y[num_of_remaining_points - 1];

        double ref_x_prev = previous_path_x[num_of_remaining_points - 2];
        double ref_y_prev = previous_path_y[num_of_remaining_points - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        anchor_pts_x.push_back(ref_x_prev);
        anchor_pts_x.push_back(ref_x);

        anchor_pts_y.push_back(ref_y_prev);
        anchor_pts_y.push_back(ref_y);
    }

    // Add 30m evenly spaced anchor points converted from Frenet coordinates to XY
    vector<double> next_wp0 = getXY(trajectory_starting_s + 30, (2 + lane_width * car.lane), road_map.waypoints_s, road_map.waypoints_x, road_map.waypoints_y);
    vector<double> next_wp1 = getXY(trajectory_starting_s + 60, (2 + lane_width * car.lane), road_map.waypoints_s, road_map.waypoints_x, road_map.waypoints_y);
    vector<double> next_wp2 = getXY(trajectory_starting_s + 90, (2 + lane_width * car.lane), road_map.waypoints_s, road_map.waypoints_x, road_map.waypoints_y);

    anchor_pts_x.push_back(next_wp0[0]);
    anchor_pts_x.push_back(next_wp1[0]);
    anchor_pts_x.push_back(next_wp2[0]);

    anchor_pts_y.push_back(next_wp0[1]);
    anchor_pts_y.push_back(next_wp1[1]);
    anchor_pts_y.push_back(next_wp2[1]);

    // Transform to have car or end of previous path as origin
    for (int i = 0; i < anchor_pts_x.size(); i++)
    {
        double shift_x = anchor_pts_x[i] - ref_x;
        double shift_y = anchor_pts_y[i] - ref_y;

        anchor_pts_x[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
        anchor_pts_y[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    }

    // Create spline from anchor points
    tk::spline s;
    s.set_points(anchor_pts_x, anchor_pts_y);

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    // Start with points remaining from previous path
    for (int i = 0; i < num_of_remaining_points; i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Fill up rest of path with interpolated values
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);
    double x_add_on = 0;

    for (int i = 1; i <= 50 - num_of_remaining_points; i++)
    {
        // Calculate step size of spline for desired speed
        double step_size = target_dist / (TIME_STEP * car.current_speed * MPH_TO_MPS);
        double x_point = x_add_on + target_x / step_size;
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

    return trajectory;
}

double Planner::lane_change_cost(string state, Vehicle car, vector<vector<double>> sensor_data){

    int desired_lane = 1;
    
    if (state == "KEEP_LANE")
    {
        return 0.0;
    }
    else if (state == "LANE_CHANGE_LEFT"){
        desired_lane = car.lane - 1;
    }
    else if (state == "LANE_CHANGE_RIGHT"){
        desired_lane = car.lane + 1;
    }

    int lane_change_cost = 2.0;
    double lane_edge_left = lane_width * desired_lane;
    double lane_edge_right = lane_width * (desired_lane + 1);

    for (int i = 0; i < sensor_data.size(); i++) {
        // data for ith car
        float vehicle_d = sensor_data[i][6];

        // check if car is in our lane
        if (vehicle_d > lane_edge_left && vehicle_d < lane_edge_right){

            double vx = sensor_data[i][3];
            double vy = sensor_data[i][4];
            double vehicle_speed = sqrt(vx * vx + vy * vy) * MPS_TO_MPH; // mph
            double vehicle_s = sensor_data[i][5];
            double speed_diff = abs(car.current_speed - vehicle_speed);

            bool safe_to_change;
            if (vehicle_s > car.s){
                if (car.current_speed > vehicle_speed){
                    safe_to_change = abs(vehicle_s - car.s) > following_time * 0.66 * (car.current_speed + speed_diff) * MPH_TO_MPS;
                }
                else{
                    safe_to_change = abs(vehicle_s - car.s) > following_time * 0.33 * (car.current_speed - speed_diff) * MPH_TO_MPS;
                }
            }
            else{
                if (car.current_speed > vehicle_speed){
                    safe_to_change = abs(vehicle_s - car.s) > following_time * 0.33 * (car.current_speed - speed_diff) * MPH_TO_MPS;
                }
                else{
                    safe_to_change = abs(vehicle_s - car.s) > following_time * 0.66 * (car.current_speed + speed_diff) * MPH_TO_MPS;
                }
            }

            if (!safe_to_change){
                lane_change_cost = 99999;
                break;
            }
            
        }
    }

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
    double lane_edge_left = lane_width * desired_lane;
    double lane_edge_right = lane_width * (desired_lane + 1);

    for (int i = 0; i < sensor_data.size(); i++) {
        // data for ith car
        float vehicle_d = sensor_data[i][6];
        double vehicle_s = sensor_data[i][5];

        // check if car is in our lane and ahead of us but not farther than 60m
        if (vehicle_d > lane_edge_left && vehicle_d < lane_edge_right && vehicle_s > car.s && vehicle_s - car.s < 60){

            double vx = sensor_data[i][3];
            double vy = sensor_data[i][4];
            double vehicle_speed = sqrt(vx * vx + vy * vy);
            average_speed += vehicle_speed;
            num_of_relevant_cars++;
            
        }
    }

    average_speed = (num_of_relevant_cars > 0) ? (average_speed * MPS_TO_MPH / num_of_relevant_cars) : max_speed;
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

    double lane_edge_left = lane_width * desired_lane;
    double lane_edge_right = lane_width * (desired_lane + 1);

    double num_of_vehicles = 0.0;

        for (int i = 0; i < sensor_data.size(); i++) {
            // data for ith car
            float vehicle_d = sensor_data[i][6];
            float vehicle_s = sensor_data[i][5];

            // check if car is in our lane
            if (vehicle_d > lane_edge_left && vehicle_d < lane_edge_right && vehicle_s > car.s && vehicle_s - car.s < 120){
                num_of_vehicles++;       
            }
        }

    return num_of_vehicles;

}

