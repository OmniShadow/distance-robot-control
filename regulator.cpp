/*
    DISCRETE-TIME REGULATOR:

    y[k] = 4.6129 * u[k]  -  3.8864 * u[k-1]  + 0.7 * y[k-1]


    LEGENDA:
    y is R output -> Meca next velocity
    u is R input -> error = reference - measured distance

*/

#include "distance_sensor/include/DistanceSensor.hpp"
#include "distance_sensor/include/InfraredSensor.hpp"
#include "meca500_ethercat_cpp/Robot.hpp"
#include "csvlogger/CsvLogger.hpp"
#include <unistd.h>
#include <iostream>
#include <math.h>
#include <chrono>
#include <Regolatore.cpp>

/*costants*/
#define DEFAULT_SAMPLE_TIME 0.02               // sampling period in seconds
#define DEFAULTREFERENCE_mm -50 // reference (desired distance Meca-Obstacle)

using namespace std;

/*GLOBAL*/
float reference_user = DEFAULTREFERENCE_mm; // specific choice by user

/*sensor variables*/
InfraredSensor sensor(InfraredSensor::USER_INPUT);
float m = 1;
float q = 0;
float Tc_s = DEFAULT_SAMPLE_TIME;

Regolatore *regolatore = nullptr;

/*FUNCTIONS*/
void menu(int n_par, char *par[]);             // manage user input from cmd
uint64_t getCurrentTimeMicros();               // return current time in microseconds
void delayMicroseconds(uint64_t microseconds); // pause execution

void setupRegulator()
{
    double pole_1 = 0.6;
    double zero_1 = 0.7967;
    double gain = 1.6334;
    vector<double> input_coeff{gain, -gain * zero_1};
    vector<double> output_coeff{2 * pole_1, -pole_1 * pole_1};
    regolatore = new Regolatore(output_coeff, input_coeff);
}

int main(int argc, char *argv[])
{
    /*menu control and sensor initialisation*/
    menu(argc, argv);
    /*sensor setup*/
    sensor.useCalibrationCurve(m, q);

    setupRegulator();

    /*robot, setup*/
    Robot robot(30, 200, 5000, "eth0", 0.0, 10);
    robot.reset_error();
    // robot.main();
    robot.set_conf(1, 1, -1);
    robot.move_pose(115, -170, 120, 90, 90, 0); // bring Meca to 0_position
    // robot.print_pose();

    /*file to write data, setup*/
    CsvLogger data_test("test_closed_loop/data_test.csv");
    data_test.write("time,reference,position,measured_distance,error,velocity_control\n"); // if !take_data -> empy file

    /*time variables setup*/
    uint64_t t0, start;
    float currentTime = 0;
    uint64_t delayInterval;

    /*interpolation variables*/
    /*variable for interpolation
        reference_user: desired distance meca - obstacle                            5 cm default
        reference_distance: reference variable  (desidered to be reference_user)     5 cm default
        interpolate: flag to smooth reference distance when required
    */
    float reference_distance = DEFAULTREFERENCE_mm;
    bool interpolate = true;
    float starting_reference = -sensor.getDistanceInMillimeters();
    float rise_time = 0.5;
    float slope = (reference_user - starting_reference) / rise_time;
    float interpolate_time = 0;
    bool out_of_range = false;

    /*other storage variables*/
    float currentDistance;
    float velocity[] = {0, 0, 0, 0, 0, 0};

    /*process variables and state variables*/
    float error;      // u[k]
    float output;      // y[k]
    /*control*/
    while (true)
    {
        start = getCurrentTimeMicros();

        /*compute distance*/
        currentDistance = -sensor.getDistanceInMillimeters();

        /*OUT OF RANGE CASE (example: obstacle removed) */
        if (currentDistance < -200)
        {
            cout << "Sensor out of range.. stopping robot\n";
            cout << "Waiting for Obstacle in range..\n";

            /*stop Meca*/
            vvelocity[0] = 0;
            regolatore->reset();
            robot.move_lin_vel_wrf(velocity);

            /*wait for obstacle.*/

            while (currentDistance < -200)
            {
                start = getCurrentTimeMicros();
                currentDistance = -sensor.getDistanceInMillimeters();

                /*export data*/
                data_test << currentTime;
                data_test << reference_distance;
                data_test << robot.get_position();
                data_test << currentDistance;
                data_test << 0;
                data_test << 0;
                data_test.end_row();

                /*compute dalay*/
                delayInterval = Tc_s * 1e6 - (getCurrentTimeMicros() - start);
                delayMicroseconds(delayInterval);
                currentTime += Tc_s;
            }

            cout << "Obstacle in range.. resuming control\n";

            /*new interpolation needed, preparation (see interpolation)*/
            interpolate = true;
            starting_reference = currentDistance;
            slope = (reference_user - starting_reference) / rise_time;
            interpolate_time = currentTime;
        }

        /* interpolation */
        if (interpolate)
        {
            reference_distance = slope * (currentTime - interpolate_time) + starting_reference;
            if ((slope > 0 && reference_distance >= reference_user) || (slope <= 0 && reference_distance <= reference_user))
            {
                reference_distance = reference_user;
                interpolate = false;
            }
        }

        /* computing */
        error = reference_distance - currentDistance;
        output = regolatore->calculate_output(error);

        /* safety control: checking robot position limits */
        if (robot.get_position() >= robot.POS_LIMIT_SUP)
        {
            if (output > 0) // if velocity is positive
            {
                output = 0;
            }
        }
        else if (robot.get_position() <= robot.POS_LIMIT_INF)
        {
            if (output < 0) // if velocity is negative
            {
                output = 0;
            }
        }

        /*give meca velocity command*/
        velocity[0] = output;
        robot.move_lin_vel_wrf(velocity);

        /*export data*/
        // "time,reference,position,measured_distance,error,velocity_control"
        data_test << currentTime;
        data_test << reference_distance;
        data_test << robot.get_position();
        data_test << currentDistance;
        data_test << error;
        data_test << output;
        data_test.end_row();

        /*delay*/
        delayInterval = Tc_s * 1e6 - (getCurrentTimeMicros() - start);
        delayMicroseconds(delayInterval); // delay by time remaining
        currentTime += Tc_s;          // increse time by Tc_s for reference smoothing
    }
}

void menu(int n_par, char *par[])
{
    /*
        cmd line ->
        regolatore
        regolatore distance
        regolatore distance m q
    */

    /* if distance_reference is passed and correct, set, else default*/
    if (n_par >= 2)
    {
        reference_user = -atof(par[1]);
    }

    /* if calibration parameters are passed and correct, set, else default*/
    if (n_par >= 4)
    {
        m = atof(par[2]);
        q = atof(par[3]);
    }
}

uint64_t getCurrentTimeMicros()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

void delayMicroseconds(uint64_t microseconds)
{
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start + std::chrono::microseconds(microseconds);

    while (std::chrono::high_resolution_clock::now() < end)
    {
        // Busy-wait loop
    }
}
