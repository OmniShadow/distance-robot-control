#ifndef _READ_SENSOR_H_
#define _READ_SENSOR_H_


#include "serial.h"

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <unistd.h> // write(), read(), close()

// Private headers


using std::string;
using std::exception;
using std::endl;
using std::vector;

/* Tactile Sensor Offset Samples Number */
#define TACTILE_OFFSET_NUM_SAMPLES 50U

/* Sensor_t Structure definition */
typedef struct
{
  uint8_t              SensorID;
  std::string          SensorName; 
  std::string          SerialPort;
  uint8_t              NumSensingElements;
  std::vector<float>   LastSensorReadingsRAW;
  std::vector<float>   Offset;
} Sensor_t;

/**
* @Brief This routine enumerates all Serial Ports of the PC.
* The output variables are:
* @li DeviceName: a vector with the device name.
* Returns the number of the found devices.
* IMPORTANT: Only the ttyUSB* and ttyACM* devices will be enumerated.
*/ 
uint8_t enumerate_ports(vector<string>& deviceName);

/**
* @Brief This routine inizializes the sensors attached to the Serial Ports.
* It enumerates all the serial ports and if a compatible port is available checks
* the presence of a Tactile or Proximity sensor reading the sensor ID.
*/ 
void init_sensor(void);

/**
* @Brief This routine inizializes the sensors attached to the Serial Port
* passed as input argument.
* It checks the presence of a Tactile or Proximity sensor reading the sensor ID.
*/ 
void init_sensor(string SerialPortName);

/**
* @Brief This routine allows to the user to select the sensor to use.
* If the input parameter 'SerialPortSpecified' is TRUE then the routine 
* simply configures the sensor data structure for the sensor attached to 
* the Serial Port specified by the user at the node instantiation. 
* If the input parameter 'SerialPortSpecified' is TRUE then the routine allows
* to choose a sensor between the sensors listed in _SensorList. 
* If _SelectedSensorID == 255U then no sensor is available.
*/ 
void select_sensor(vector<Sensor_t> sensorList, bool SerialPortSpecified);

#endif /* _READ_SENSOR_H_ */
