#ifndef DISTANCESENSOR_HPP
#define DISTANCESENSOR_HPP
#include <pigpio.h>
class DistanceSensor
{
public:
    virtual ~DistanceSensor() {}

    // Virtual method to get distance in meters
    virtual float getDistanceInMeters() = 0;

    // Virtual method to get distance in centimeters
    virtual float getDistanceInCentimeters() = 0;

    // Virtual method to get distance in millimeters
    virtual float getDistanceInMillimeters() = 0;

    // Virtual method to use calibration
    virtual void useCalibrationCurve(float m, float q) = 0;
};

#endif // DISTANCESENSOR_HPP
