#ifndef VEHICLE_H
#define VEHICLE_H

#include <string>

class Vehicle {

    private:
        

    public:
        Vehicle();
        Vehicle(double x, double y, double s, double d, double yaw, double speed);
        double x;
        double y;
        double s;
        double d;
        double yaw;
        double speed;
        
        ~Vehicle();

};

#endif // VEHICLE_H