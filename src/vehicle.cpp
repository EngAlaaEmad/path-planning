#include "vehicle.h"

Vehicle::Vehicle() {}
Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed):
                x(x), y(y), s(s), d(d), yaw(yaw), speed(speed){}

Vehicle::~Vehicle() {}