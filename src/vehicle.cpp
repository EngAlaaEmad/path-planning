#include "vehicle.h"
#include <iostream>

Vehicle::Vehicle() {}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed, double desired_speed):
                x(x), y(y), s(s), d(d), yaw(yaw), speed(speed), desired_speed(desired_speed){}

Vehicle::~Vehicle() {}