#ifndef ULTRASONICSENSOR_HPP
#define ULTRASONICSENSOR_HPP

#include "DistanceSensor.hpp"
#include <pigpio.h>
#include <stdint.h>

/*CONSTANTS*/
#define SOUND_SPEED_MS 343
#define MAX_SENSOR_DISTANCE_CM 200 // Maximum sensor distance can be as high as 500cm, no reason to wait for ping longer than sound takes to travel this distance and back. Default=500
#define US_ROUNDTRIP_CM 57         // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space. Default=57
#define US_ROUNDTRIP_IN 146        // Microseconds (uS) it takes sound to travel round-trip 1 inch (2 inches total), uses integer to save compiled code space. Default=146
#define TRIGGER_WIDTH 20
#define MAX_SENSOR_DELAY 5800 // Maximum uS it takes for sensor to start the ping. Default=5800

class UltrasonicSensor : public DistanceSensor
{

private:
    /*PRIVATE ATTRIBUTES*/
    unsigned int MAX_SENSOR_DISTANCE_MM;
    uint8_t attachedTrigPin;
    uint8_t attachedEchoPin;
    unsigned int _maxEchoTime;
    bool calibrationEnabled = false;
    float m_cal;
    float q_cal;

    /*PRIVATE METHODS*/
    /**
     * This function return the calibrated measure based on the spoilt measure to calibrate ad the calibration line
     */
    float getCalibratedDistance(float spoiltMeasure);

public:
    /*CONSTRUCTOR*/
    UltrasonicSensor(uint8_t attachedTrigPin, uint8_t attachedEchoPin);

    /*PUBLIC METHODS*/
    // Implementation of virtual methods to get distance in meters, centimeters, and millimeters
    float getDistanceInMeters() override;
    float getDistanceInCentimeters() override;
    float getDistanceInMillimeters() override;
    void useCalibrationCurve(float m, float q) override;
};

#endif // ULTRASONICSENSOR_HPP
