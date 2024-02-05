#include "DistanceSensor.hpp"
#include "UltrasonicSensor.hpp"

/*CONSTRUCTOR*/
UltrasonicSensor::UltrasonicSensor(
    uint8_t attachedTrigPin,
    uint8_t attachedEchoPin)
    : attachedTrigPin(attachedTrigPin),
      attachedEchoPin(attachedEchoPin)
{
    gpioInitialise();
    _maxEchoTime = (unsigned int)MAX_SENSOR_DISTANCE_CM * US_ROUNDTRIP_CM; // tempo massimo di attesa echo in micro-secondi us
    gpioSetMode(attachedTrigPin, PI_OUTPUT);
    gpioSetMode(attachedEchoPin, PI_INPUT);

    /*!!!!!!!!!!!!!!!!aggiunta carica curva di calibrazione...
        calibrationLine. m = ...
        calibrationLine. q = ...
    */
}

/*PUBLIC METHODS*/
/*IMPORTANT: Returns -1 if the sensor timed out else it returns the correct distance*/
float UltrasonicSensor::getDistanceInMeters()
{
    gpioWrite(attachedTrigPin, 0);

    // Invia un impulso di trigger di durata TRIGGER_WIDTH
    gpioTrigger(attachedTrigPin, TRIGGER_WIDTH, 1);
    // calcolo il massimo valore di attesa per l'ecbo
    unsigned long _max_time = gpioTick() + _maxEchoTime + MAX_SENSOR_DELAY;

    // attendo che il pin di echo si alzi
    while (!gpioRead(attachedEchoPin) && gpioTick() < _max_time)
    {
    }

    if (gpioTick() >= _max_time)
    {
        // se il tempo trascorso supera il valore massimo vado in time out
        return -1;
    }
    uint32_t tempo = gpioTick();
    while (gpioRead(attachedEchoPin))
    {
    }

    // calcolo il tempo trascorso tra trigger e ricezione di echo (ping time)
    uint32_t elapsed_time = gpioTick() - tempo;
    // calcolo la distanza in base al tempo trascorso
    float distance = elapsed_time / 1000000.0 * SOUND_SPEED_MS / 2.0;

    /*ritorno la misura calibrata o non calibrata*/
    if (calibrationEnabled)
        return getCalibratedDistance(distance);
    else
        return distance;
}
float UltrasonicSensor::getDistanceInCentimeters()
{
    return getDistanceInMeters() * 100;
}
float UltrasonicSensor::getDistanceInMillimeters()
{
    return getDistanceInMeters() * 1000;
}

void UltrasonicSensor::useCalibrationCurve(float m, float q)
{
    m_cal = m;
    q_cal = q;
    calibrationEnabled = true;
}

/*PRIVATE METHODS*/
float UltrasonicSensor::getCalibratedDistance(float spoiltMeasure)
{
    return (spoiltMeasure - q_cal) / m_cal;
}
