
#include "DistanceSensor.hpp"
#include "InfraredSensor.hpp"
#include <string>
#include <algorithm>
#include <string.h>
#include <vector>
#include <cstdint>
#include <iostream>
#include <serial/serial.h>

using std::cerr;
using std::cout;
using std::endl;
using std::exception;
using std::string;
using std::vector;

using namespace std;

/*CONSTRUCTOR*/
InfraredSensor::InfraredSensor(int argc)
{
    // calibrationLine = calibration_line;
    char enable_all[] = "utlr";
    char *argv[] = {"test", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"};
    char *to_enable;
    /* Sensor Initialization */
    if (argc == 1U)
    {
        init_sensor();
        select_sensor(_SensorList, false);
        to_enable = enable_all;
    }
    else if (argc == 2U)
    {
        if (strlen(argv[1]) > 4)
        {
            string userInput = argv[1];
            init_sensor(userInput);
            select_sensor(_SensorList, true);
            to_enable = enable_all;
        }
        else
        {
            init_sensor();
            select_sensor(_SensorList, false);
            to_enable = argv[1];
        }
    }
    else if (argc == 3U)
    {
        init_sensor(argv[1]);
        select_sensor(_SensorList, true);
        to_enable = argv[2];
    }

    /* Set Publisher and Rate for P--- Sensors */
    uint16_t rate = 0U;
    if (strstr((char *)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "P"))
    {
        size_t bytes_wrote = _SensorSerial->write(to_enable);
        string topicName_raw = "ProximityData" + _SensorList.at(_SelectedSensorID).SerialPort + "_" + _SensorList.at(_SelectedSensorID).SensorName;
        replace(topicName_raw.begin(), topicName_raw.end(), '/', '_');
        rate = 50U;
    }

}

/*PUBLIC*/
float InfraredSensor::getDistanceInMeters()
{
    return getDistanceInMillimeters() / 1000;
}
float InfraredSensor::getDistanceInCentimeters()
{
    return getDistanceInMillimeters() / 10;
}
float InfraredSensor::getDistanceInMillimeters() // here is implemented the calibration
{
    if (calibrationEnabled)
        return getCalibratedDistance(getDistanceInMillimetersVector()[0]);
    else
        return getDistanceInMillimetersVector()[0];
}

void InfraredSensor::useCalibrationCurve(float m, float q)
{
    m_cal = m;
    q_cal = q;
    calibrationEnabled = true;
}

/*PRIVATE*/

float InfraredSensor::getCalibratedDistance(float spoiltMeasure)
{
    return (spoiltMeasure - q_cal) / m_cal;
}

vector<uint8_t> InfraredSensor::getDistanceInMillimetersVector()
{
    /*IMPORTANT: INDEPENDENT FROM CALIBRATION!!!! (NOT CALIBRATED)*/

    /* Temporary variable for RAW Sensor data */
    vector<uint8_t> dataRaw;
    dataRaw.clear();
    _SensorList.at(_SelectedSensorID).LastSensorReadingsRAW.clear();

    /* Sensor readings for P--- Sensors */
    if (strstr((char *)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "P"))
    {
        /* Request sensor data */
        size_t bytes_wrote = _SensorSerial->write("a");

        _SensorSerial->read(dataRaw, _SensorList.at(_SelectedSensorID).NumSensingElements);

        /* Fill data buffer and ROS msg */
        for (int k = 0U; k < _SensorList.at(_SelectedSensorID).NumSensingElements; k++)
        {
            // printf("dimensione dataRaw = %d\n", dataRaw.size()); // DEBUG
            _SensorList.at(_SelectedSensorID).LastSensorReadingsRAW.push_back(dataRaw.at(k));
            /* Print sensor data values on the screen */
            // printf("Data %d: %d mm\r\n", k + 1, dataRaw.at(k));
        }
        return dataRaw;
    }
    return dataRaw;
}

uint8_t InfraredSensor::enumerate_ports(vector<string> &deviceName)
{
    char *nameCheck1;
    char *nameCheck2;
    const char *name1 = "ttyUSB";
    const char *name2 = "ttyACM";

    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end())
    {
        serial::PortInfo device = *iter++;

        nameCheck1 = strstr((char *)device.port.c_str(), name1);
        nameCheck2 = strstr((char *)device.port.c_str(), name2);

        if (nameCheck1 || nameCheck2)
        {
            // printf( "(%s, %s, %s) %d\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str(), distance(devices_found.begin(), iter) );
            // printf( "(%s, %s, %s) ID: %d\n", devices_found.at(32).port.c_str() , device.description.c_str(), device.hardware_id.c_str(), distance(devices_found.begin(), DeviceID) );
            deviceName.push_back(device.port);
        }
    }

    /* Return the number of the found Serial devices */
    return deviceName.size();
}

/**
 * @Brief This routine inizializes the sensors attached to the Serial Ports.
 * It enumerates all the serial ports and if a compatible port is available checks
 * the presence of a Tactile or Proximity sensor reading the sensor ID.
 */
void InfraredSensor::init_sensor(void)
{
    /* Private variables */
    uint8_t serialPortNum = 0U;

    /* Serial Port enumeration */
    serialPortNum = enumerate_ports(_DeviceName);

    /* No Serial port found. Exit with error. */
    if (!serialPortNum)
    {
        printf("Error: no Compatible Serial Ports found in the System. Please, make sure a ttyUSB* or a ttyACM* device is connected.\r\n");
        exit(-1);
    }
    /* Serial Port found. Print the ports. */
    else
    {
        printf("Compatible Serial Ports found on the System: %d\r\nChecking for compatible Sensor Devices...\r\n", serialPortNum);

        /* List the Device Ports */
        uint8_t sensorID = 0U;
        for (uint8_t k = 0U; k < serialPortNum; k++)
        {
            /* Open Serial Port */
            /* port, baudrate, timeout in milliseconds */
            serial::Serial my_serial(_DeviceName.at(k), 1000000U, serial::Timeout::simpleTimeout(1000U));

            /* Check if a sensor is connected to the serial port */
            size_t bytes_wrote = my_serial.write("b");

            if (bytes_wrote > 0U)
            {
                string result = my_serial.read(4U);

                if (4U == result.length())
                {
                    /* Print header of sensor list */
                    if (0U == sensorID)
                    {
                        printf("\r\nDevice List:\r\n");
                        printf("### PortID [#] - Port Name ## Sensor Name ###\r\n");
                    }
                    else
                    {
                        ;
                    }

                    /* Fill sensor information */
                    Sensor_t currentSensor;
                    currentSensor.SensorID = sensorID;
                    currentSensor.SensorName = result;
                    currentSensor.SerialPort = _DeviceName.at(k);

                    _SensorList.push_back(currentSensor);

                    /* Print Device Information */
                    printf("### [%d] - %s ## ", currentSensor.SensorID, currentSensor.SerialPort.c_str());
                    printf("%s ###\r\n", currentSensor.SensorName.c_str());

                    /* Update sensor ID count */
                    sensorID++;
                }
                else
                {
                    ;
                }
            }
            else
            {
                ;
            }

            /* Close Serial Port */
            my_serial.close();
        }

        /* If at this point sensorID == 0U, then no sensor devices have been found */
        if (0U == sensorID)
        {
            _SelectedSensorID = 255U;
        }
        else
        {
            _SelectedSensorID = 254U;
        }
    }
}

/**
 * @Brief This routine inizializes the sensors attached to the Serial Port
 * passed as input argument.
 * It checks the presence of a Tactile or Proximity sensor reading the sensor ID.
 */
void InfraredSensor::init_sensor(string SerialPortName)
{
    uint8_t sensorID = 0U;

    try
    {
        /* Open Serial Port */
        /* port, baudrate, timeout in milliseconds */
        serial::Serial my_serial(SerialPortName, 1000000U, serial::Timeout::simpleTimeout(1000U));

        /* Check if a sensor is connected to the serial port */
        size_t bytes_wrote = my_serial.write("b");
        string result = my_serial.read(4U);

        if (4U == result.length())
        {
            /* Fill sensor information */
            Sensor_t currentSensor;
            currentSensor.SensorID = sensorID;
            currentSensor.SensorName = result;
            currentSensor.SerialPort = SerialPortName;

            _SensorList.push_back(currentSensor);

            /* Update sensor ID count */
            sensorID++;
        }
        else
        {
            ;
        }

        /* Close Serial Port */
        my_serial.close();
    }
    catch (std::exception &e)
    {
        printf("Error: cannot open the specified Serial Port. Port syntax: /dev/ttyUSB* or /dev/ttyACM*.\r\n");
        exit(-1);
    }

    /* If at this point sensorID == 0U, then no sensor devices have been found */
    if (0U == sensorID)
    {
        _SelectedSensorID = 255U;
    }
    else
    {
        _SelectedSensorID = 0U;
    }
}

/**
 * @Brief This routine allows to the user to select the sensor to use.
 * If the input parameter 'SerialPortSpecified' is TRUE then the routine
 * simply configures the sensor data structure for the sensor attached to
 * the Serial Port specified by the user at the node instantiation.
 * If the input parameter 'SerialPortSpecified' is TRUE then the routine allows
 * to choose a sensor between the sensors listed in _SensorList.
 * If _SelectedSensorID == 255U then no sensor is available.
 */
void InfraredSensor::select_sensor(vector<Sensor_t> sensorList, bool SerialPortSpecified)
{
    if (!SerialPortSpecified)
    {
        if (255U == _SelectedSensorID)
        {
            printf("\r\nNo sensors are available on the detected Serial Ports.\r\n");
            exit(-1);
        }
        else
        {
            while (1)
            {
                printf("\r\nPlease, select the Sensor to use by inserting the PortID #: ");
                scanf("%d", &_SelectedSensorID);

                if (_SelectedSensorID < _SensorList.size() && _SelectedSensorID >= 0)
                {
                    break;
                }
                else
                {
                    printf("A wrong PortID has been selected.\r\n");
                }
            }

            printf("Selected Sensor: %d\r\n", _SelectedSensorID);

            /* Open Serial Port */
            _SensorSerial = new serial::Serial(_SensorList.at(_SelectedSensorID).SerialPort, 1000000U, serial::Timeout::simpleTimeout(1000U));

            /* Set low latency option */
            char command[100] = {};
            strcat(command, "setserial ");
            strcat(command, _SensorList.at(_SelectedSensorID).SerialPort.c_str());
            strcat(command, " low_latency");
            system(command);

            /**
             * Read the number of sensing elements.
             * If the sensor is a proximity sensor, then ask for the number of
             * sensing elements, otherwise set it to 25.
             */
            if (strstr((char *)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "P"))
            {
                size_t bytes_wrote = _SensorSerial->write("c");
                _SensorSerial->read(&_SensorList.at(_SelectedSensorID).NumSensingElements, 1U);
            }
            else if (strstr((char *)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "F"))
            {
                _SensorList.at(_SelectedSensorID).NumSensingElements = 25U;
            }
            else if (strstr((char *)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "S"))
            {
                _SensorList.at(_SelectedSensorID).NumSensingElements = 12U;
            }
            else
            {
                ;
            }

            if (_SensorList.at(_SelectedSensorID).NumSensingElements > 0U)
            {
                printf("Number of the Sensor Sensing Elements: %d\r\n", _SensorList.at(_SelectedSensorID).NumSensingElements);
            }
            else
            {
                printf("No Sensing Elements available on the selected Sensor.");
                exit(-1);
            }
        }
    }
    else if (SerialPortSpecified)
    {
        if (255U == _SelectedSensorID)
        {
            printf("\r\nNo sensors are available on the specified Serial Port.\r\n");
            exit(-1);
        }
        else
        {
            printf("Serial Port %s is used for the current sensor.\r\n", _SensorList.at(_SelectedSensorID).SerialPort.c_str());

            /* Open Serial Port */
            _SensorSerial = new serial::Serial(_SensorList.at(_SelectedSensorID).SerialPort, 1000000U, serial::Timeout::simpleTimeout(1000U));

            /* Set low latency option */
            char command[100] = {};
            strcat(command, "setserial ");
            strcat(command, _SensorList.at(_SelectedSensorID).SerialPort.c_str());
            strcat(command, " low_latency");
            system(command);

            /**
             * Read the number of sensing elements.
             * If the sensor is a proximity sensor, then ask for the number of
             * sensing elements, otherwise set it to 25.
             */
            if (strstr((char *)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "P"))
            {
                size_t bytes_wrote = _SensorSerial->write("c");
                _SensorSerial->read(&_SensorList.at(_SelectedSensorID).NumSensingElements, 1U);
            }
            else if (strstr((char *)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "F"))
            {
                _SensorList.at(_SelectedSensorID).NumSensingElements = 25U;
            }
            else if (strstr((char *)_SensorList.at(_SelectedSensorID).SensorName.c_str(), "S"))
            {
                _SensorList.at(_SelectedSensorID).NumSensingElements = 12U;
            }
            else
            {
                ;
            }

            if (_SensorList.at(_SelectedSensorID).NumSensingElements > 0U)
            {
                printf("Number of the Sensor Sensing Elements: %d\r\n", _SensorList.at(_SelectedSensorID).NumSensingElements);
            }
            else
            {
                printf("No Sensing Elements available on the selected Sensor.");
                exit(-1);
            }
        }
    }
    else
    {
        ;
    }
}