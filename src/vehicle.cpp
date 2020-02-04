#include "vehicle.h"
#include <math.h>
#include <vector>
#include <iostream>

using std::vector;

Vehicle::Vehicle() {}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed, double desired_speed):
                x(x), y(y), s(s), d(d), yaw(yaw), speed(speed), desired_speed(desired_speed){}

void Vehicle::set_speed(double ref_speed, double MAX_SPEED, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, vector<vector<double>> sensor_data){

  int prev_size = previous_path_x.size();
  if (prev_size > 0)
  {
    this->s = end_path_s;
  }

  bool car_ahead = false;

  for (int i = 0; i < sensor_data.size(); i++)
  {
    // data for ith car
    float d = sensor_data[i][6];

    // check if car is in our lane
    if (d < (2 + 4 * this->lane + 2) && d > (2 + 4 * this->lane - 2))
    {
      double vx = sensor_data[i][3];
      double vy = sensor_data[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_data[i][5];

      // project cars s value out to end of path (future pos)
      check_car_s += (double)prev_size * 0.02 * check_speed;

      if ((check_car_s > this->s) && (check_car_s - this->s < 2 * (this->desired_speed) * 0.447))
      {
        car_ahead = true;
        ref_speed = check_speed;
      }
    }
  }

  // Slow down if too close
  if (car_ahead == true && this->desired_speed > ref_speed)
  {
    this->desired_speed -= 0.224;
  }
  // Otherwise speed up incrementally
  else if (this->desired_speed < MAX_SPEED)
  {
    this->desired_speed += 0.224;
  }

}

Vehicle::~Vehicle() {}