#include "distance_sensor/include/DistanceSensor.hpp"
#include "distance_sensor/include/InfraredSensor.hpp"
#include "meca500_ethercat_cpp/Robot.hpp"
#include "csvlogger/CsvLogger.hpp"
#include <Regolatore.cpp>
#include <vector>
#include <unistd.h>
#include <iostream>
#include <math.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <string>
#include <map>
#include <cstring>
#include <sstream>
#include <iomanip>

#define HELP_COMMAND "help"
#define STOP_COMMAND "stop"
#define REFERENCE_COMMAND "rif"
#define CALIBRATION_CURVE_COMMAND "cal"

#define optionWidth 60
#define descriptionWidth 60
#define message_length 30

#define SAMPLING_TIME 0.02
#define DEFAULT_REFERENCE_mm 50

using namespace std;
stringstream helpMessage;

stringstream stopMessage;

stringstream refMessage;

stringstream calMessage;

struct OptionHandler
{
    using Handler = string (*)(string);

    OptionHandler() : handler(nullptr), helpMessage("") {}
    OptionHandler(Handler func, const string &helpMsg)
        : handler(func), helpMessage(helpMsg) {}

    Handler handler;
    string helpMessage;
};

map<string, OptionHandler> optionHandlers;

uint64_t getCurrentTimeMicros(); // return current time in microseconds

void setup();
void setupSensor();
void setupRobot();
void setupRegulator();
void setupCsvLogger();
void setupHelpMessages();

vector<std::string> splitString(const string &input);
map<string, string> parseOptionTokens(int argc, char *argv[]);

string handleHelp(string value);
string handleStop(string value);
string handleRef(string value);
string handleCalibration(string value);

void writeDataToCsv(float time, float reference, float position, float measured_distance, float error, float velocity_control);

std::atomic<float> distanzaRiferimento(DEFAULT_REFERENCE_mm);
bool isRunning = true;

InfraredSensor *infraredSensor = nullptr;
Robot *robot = nullptr;
Regolatore *regolatore = nullptr;
CsvLogger *data_test = nullptr;

float currentDistance;

float reference_distance = DEFAULT_REFERENCE_mm;
bool interpolate = true;
float starting_reference = 0;
float rise_time = 0.5;
float slope = (distanzaRiferimento - starting_reference) / rise_time;
float interpolate_time = 0;
bool out_of_range = false;

float input = 0;
float output = 0;

int main(int argc, char const *argv[])
{
    setup();
    // Create two threads
    std::thread controlLoop();
    std::thread riceviOpzioni();

    // Wait for threads to finish execution
    controllo.join();
    riceviOpzioni.join();

    return 0;
}

void setup()
{
    setupSensor();
    setupRobot();
    setupRegulator();
    setupCsvLogger();
    setupHelpMessages();
    setupCommandHandlers();
}

void setupSensor()
{
    infraredSensor = new InfraredSensor(InfraredSensor::USER_INPUT);
    infraredSensor->useCalibrationCurve(1, 0);
}

void setupRobot()
{
    robot = new Robot(30, 200, 5000, "eth0", 0.0, 10);
    robot->reset_error();
    robot->set_conf(1, 1, -1);
    robot->move_pose(115, -170, 120, 90, 90, 0);
}

void setupRegulator()
{
    double pole_1 = 0.6;
    double zero_1 = 0.7967;
    double gain = 1.6334;
    double input = 0;
    double output = 0;
    vector<double> input_coeff{gain, -gain * zero_1};
    vector<double> output_coeff{2 * pole_1, -pole_1 * pole_1};
    regolatore = new Regolatore(output_coeff, input_coeff);
}

void setupCsvLogger()
{
    data_test = new CsvLogger("test_closed_loop/data_test.csv");
    data_test->write("time,reference,position,measured_distance,error,velocity_control\n"); // if !take_data -> empy file
}

void writeDataToCsv(float time, float reference, float position, float measured_distance, float error, float velocity_control)
{
    data_test << current_time;
    data_test << reference_distance;
    data_test << robot.get_position();
    data_test << currentDistance;
    data_test << 0;
    data_test << 0;
    data_test.end_row();
}

void controlLoop()
{
    uint64_t t0, start;
    float current_time = 0;
    uint64_t delay_time;

    float velocity[] = {0, 0, 0, 0, 0, 0};
    while (isRunning)
    {
        start = getCurrentTimeMicros();

        /*compute distance*/
        currentDistance = infraredSensor->getDistanceInMillimeters();
        starting_reference = currentDistance;

        /*OUT OF RANGE CASE (example: obstacle removed) */
        if (currentDistance > 200)
        {
            cout << "Sensor out of range.. stopping robot\n";
            cout << "Waiting for Obstacle in range..\n";

            /*stop Meca*/
            velocity[0] = 0;
            regolatore->reset();
            robot.move_lin_vel_wrf(velocity);
            /*wait for obstacle.*/

            while (currentDistance > 200)
            {
                start = getCurrentTimeMicros();
                currentDistance = infraredSensor->getDistanceInMillimeters();

                /*export data*/
                writeDataToCsv(current_time, reference_distance, robot->get_position(), currentDistance, distanzaRiferimento - currentDistance, 0);
                /*compute dalay*/
                delay_time = Tc_s * 1e6 - (getCurrentTimeMicros() - start);
                std::this_thread::sleep_for(std::chrono::microseconds(delay_time));
                current_time += Tc_s;
            }

            cout << "Obstacle in range.. resuming control\n";

            /*new interpolation needed, preparation (see interpolation)*/
            interpolate = true;
            starting_reference = currentDistance;
            slope = (distanzaRiferimento - starting_reference) / rise_time;
            interpolate_time = current_time;
        }

        /* interpolation */
        if (interpolate)
        {
            reference_distance = slope * (current_time - interpolate_time) + starting_reference;
            if ((slope > 0 && reference_distance >= distanzaRiferimento) || (slope <= 0 && reference_distance <= reference_user))
            {
                reference_distance = distanzaRiferimento;
                interpolate = false;
            }
        }

        /* computing */
        input = reference_distance - currentDistance;
        output = regolatore->calculate_output(input);
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
        writeDataToCsv(current_time, reference_distance, robot->get_position(), currentDistance, input, output);

        /*delay*/
        delay_time = Tc_s * 1e6 - (getCurrentTimeMicros() - start);
        current_time += Tc_s; // increse time by Tc_s for reference smoothing

        std::this_thread::sleep_for(std::chrono::microseconds(delay_time)); // Print every second
    }
}

// Function for the thread that gets input from the user to update the shared value
void riceviOpzioni()
{
    string input;
    while (isRunning)
    {
        cout << "Inserisci comandi da eseguire --commandname=commandvalue [--help]: " << endl;
        getline(std::cin, input);

        vector<string> tokens = splitString(input);

        int argc = tokens.size();
        char **argv = new char *[argc];
        for (int i = 0; i < argc; ++i)
        {
            argv[i] = new char[tokens[i].size() + 1];
            std::strcpy(argv[i], tokens[i].c_str());
        }

        executeOptions(parseOptionTokens(argc, argv));
    }
}

void setupCommandHandlers()
{
    optionHandlers[HELP_COMMAND] = OptionHandler(handleHelp, helpMessage.str());
    optionHandlers[REFERENCE_COMMAND] = OptionHandler(handleRef, refMessage.str());
    optionHandlers[STOP_COMMAND] = OptionHandler(handleStop, stopMessage.str());
    optionHandlers[CALIBRATION_CURVE_COMMAND] = OptionHandler(handleCalibration, calMessage.str());
}

int executeOptions(map<string, string> options)
{
    for (auto option : options)
    {
        string optionName = option.first;
        string value = option.second;
        if (optionHandlers.count(optionName))
        {
            cout << optionHandlers[optionName].handler(value) << endl;
        }
    }

    return 0;
}
string handleHelp(string value)
{
    stringstream optionMessage;
    optionMessage << left << setw(optionWidth) << "Comandi disponibili:" << endl;

    for (auto option : optionHandlers)
    {
        cout << option.second.helpMessage;
    }
    return optionMessage.str();
}
string handleStop(string value)
{
    cout << "Stopping execution\n";
    robot->move_lin_vel_wrf({0, 0, 0, 0, 0, 0});
    robot->deactivate();
    exit(0);
    return "";
}

string handleRef(string value)
{
    stringstream optionMessage;
    optionMessage << left << setw(message_length) << "Riferimento impostato a: " << value << "\n";
    distanzaRiferimento = stof(value);
    return optionMessage.str();
}

string handleCalibration(string value)
{
    stringstream optionMessage;
    vector<float> calibration_values = parse_string_to_vector(value);

    if (infraredSensor != nullptr)
    {
        infraredSensor->useCalibrationCurve(calibration_values[0], calibration_values[1]);
    }
    optionMessage << left << setw(message_length) << "Parametri calibrazione sensore: " << value << "\n";

    return optionMessage.str();
}

void setupHelpMessages()
{
    helpMessage
        << left
        << "  --" << setw(optionWidth) << HELP_COMMAND << setw(descriptionWidth) << "Display this help message" << endl;
    refMessage
        << left
        << "  --" << REFERENCE_COMMAND << setw(optionWidth - strlen(REFERENCE_COMMAND))
        << "=Valore_riferimento_mm"
        << "Specifica un valore di riferimento in millimetri" << endl;
    stopMessage
        << left
        << "  --" << setw(optionWidth) << STOP_COMMAND << setw(descriptionWidth)
        << "Use robot for measurements" << endl;
    calMessage
        << left
        << "  --" << CALIBRATION_CURVE_COMMAND << setw(optionWidth - strlen(CALIBRATION_CURVE_COMMAND))
        << "=\"{m, q}\""
        << "Specifica i parametri di calibrazione del sensore [default {1, 0} ]" << endl;
}

vector<std::string> splitString(const string &input)
{
    vector<std::string> tokens;
    istringstream iss(input);
    string token;
    while (getline(iss, token, ' '))
    {
        tokens.push_back(token);
    }
    return tokens;
}

map<string, string> parseOptionTokens(int argc, char *argv[])
{
    map<string, string> options;

    for (int i = 1; i < argc; ++i)
    {
        string arg = argv[i];
        if (arg.substr(0, 2) == "--")
        {
            string command;
            // Found an option
            size_t pos = arg.find('=');
            if (pos != string::npos)
                command = arg.substr(2, pos - 2);
            else
                command = arg.substr(2);
            string value = (pos != string::npos) ? arg.substr(pos + 1) : "";

            options[command] = value;
        }
    }

    return options;
}