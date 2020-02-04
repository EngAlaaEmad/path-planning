#ifndef VEHICLE_H
#define VEHICLE_H

#include <string>
#include <vector>

using std::vector;
using std::string;

class Vehicle {

    private:
        

    public:
        Vehicle();
        Vehicle(double x, double y, double s, double d, double yaw, double speed, double desired_speed);
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
        double desired_speed = 0.0;
        int lane = 1;
        string state = "KEEP_LANE";
        void set_speed(double ref_speed, double MAX_SPEED, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, vector<vector<double>> sensor_data);
        ~Vehicle();

};

#endif // VEHICLE_H