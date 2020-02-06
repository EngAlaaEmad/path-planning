#ifndef VEHICLE_H
#define VEHICLE_H

#include <string>
using std::string;

class Vehicle {

    public:
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
        double current_speed;
        int lane;
        int keep_lane_cnt = 0;
        string state = "KEEP_LANE";

};

#endif // VEHICLE_H