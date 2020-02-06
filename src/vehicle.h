#ifndef VEHICLE_H
#define VEHICLE_H

#include <string>

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
        int keep_lane_cnt = 0;
        string state = "KEEP_LANE";
        ~Vehicle();

};

#endif // VEHICLE_H