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

// definizione dei comandi disponibili
#define HELP_COMMAND "help"
#define STOP_COMMAND "stop"
#define REFERENCE_COMMAND "rif"
#define CALIBRATION_CURVE_COMMAND "cal"
#define PAUSE_COMMAND "pause"

// parametri per le descrizioni dei comandi
#define optionWidth 60
#define descriptionWidth 60
#define message_length 30

// parametri per il controllo
#define SAMPLING_TIME 0.02         // Periodo di campionamento in secondi
#define SAMPLING_TIME_MICROS 20000 // Periodi di campionamento in micro secondi
#define DEFAULT_REFERENCE_mm -50   // Distanza di riferimento di default
#define INTERPOLATION_DURATION 0.5 // Durata interpolazione riferimento in secondi

using namespace std;

// Messagi relativi ai comandi disponibili
stringstream helpMessage;
stringstream stopMessage;
stringstream pauseMessage;
stringstream refMessage;
stringstream calMessage;

// Struct per la gestione dei comandi
struct OptionHandler
{
    using Handler = string (*)(string);

    OptionHandler() : handler(nullptr), helpMessage("") {}
    OptionHandler(Handler func, const string &helpMsg)
        : handler(func), helpMessage(helpMsg) {}

    Handler handler;
    string helpMessage;
};

// Mappa contenente i comandi disponibili
map<string, OptionHandler> optionHandlers;

uint64_t getCurrentTimeMicros(); // ritorna il tempo attuale in microsecondi

// Funzioni di inizializzazione
void setup();
void setupSensor();          // Setup del sensore
void setupRobot();           // Setup del Meca500
void setupRegulator();       // Setup regolatore
void setupCsvLogger();       // Setup logger per i dati di controllo
void setupHelpMessages();    // Setup per i messaggi dei comandi
void setupCommandHandlers(); // Setup dei comandi

// Funzioni per il parsing dei comandi
vector<std::string> splitString(const string &input);
map<string, string> parseOptionTokens(int argc, char *argv[]);
vector<float> parseStringToVector(string input);

// Funzione per l'esecuzione dei comandi ricevuti
int executeOptions(map<string, string> options);

// Handlers dei singoli comandi
string handleHelp(string value);
string handleStop(string value);
string handlePause(string value);
string handleRef(string value);
string handleCalibration(string value);

// Funzioni del ciclo di controllo
void handleOutOfRange();                                // Funzione che gestisce l'ostacolo fuori portata del sensore
void interpolateReference();                            // Funzione che gestisce il calcolo dell'interpolazione del riferimento
void moveRobotToPosition(vector<float> robot_position); // Wrapper del metodo Robot::move_pose
void writeDataToCsv(float time, float reference, float position, float measured_distance, float error, float velocity_control, CsvLogger &logger);

void controlLoop();   // Ciclo di controllo
void receiveCommands(); // Ciclo di ricezione dei comandi

std::atomic<float> finalReferenceDistance(DEFAULT_REFERENCE_mm); // Distanza di riferimento scelta
std::atomic<bool> isRunning(true);                                  // Flag per l'esecuzione del programma
std::atomic<bool> controlLoopActive(true);                          // Flag per l'esecuzione del ciclo di controllo

InfraredSensor *infraredSensor = nullptr; // Puntatore all'oggetto per la gestione del sensore
Robot *robot = nullptr;                   // Puntatore all'oggetto per la gestione del Meca500
Regolatore *regolatore = nullptr;         // Puntatore all'oggetto regolatore
CsvLogger *csvLogger = nullptr;              // Puntatore all'oggetto per il logging dei dati

float currentDistance; // Variabile contente la distanza attuale misurata

float currentReferenceDistance = DEFAULT_REFERENCE_mm;                                                            // Distanza di riferimento attuale per l'interpolazione
bool interpolationActive = true;                                                                                  // Flag per l'interpolazione del riferimento
float startingReferenceDistance = 0;                                                                              // Variabile usata per l'inizializzazione dell'interpolazione
float interpolationDuration = INTERPOLATION_DURATION;                                                                                   // Durata dell'interpolazione del riferimento in secondi
float interpolationSlope = (finalReferenceDistance - startingReferenceDistance) / interpolationDuration; // Pendenza della retta interpolante
float interpolationTime = 0;                                                                                      // Istante iniziale per l'inizio dell'interpolazione
bool obstacleOutOfRange = false;                                                                                          // Flag ostacolo fuori portata

uint64_t start;         // Istante di inizio controllo
float current_time = 0; // Tempo attuale in secondi
uint64_t delayDuration;    // Tempo di attesa in microsecondi

float velocity[] = {0, 0, 0, 0, 0, 0}; // Vettore per le velocità del Meca500

float error = 0;  // Errore tra riferimento e distanza misurata
float output = 0; // Iutput del regolatore

string csvDataPath; // Percorso per il salvataggio dei dati di controllp

int main(int argc, char const *argv[])
{
    // Percorso di salvataggio dati passando a riga di comando
    if (argc > 1)
    {
        csvDataPath = argv[1];
    }
    // Percorso di salvataggio di default
    else
    {
        csvDataPath = "dati_regolatore/data.csv";
    }
    // Inizializzazione delle variabili
    setup();
    // Creazione ed esecuzione dei thread
    std::thread controlLoopThread(controlLoop);
    std::thread riceviOpzioniThread(receiveCommands);

    // Aspetta che entrambi i thread terminino l'esecuzione
    controlLoopThread.join();
    riceviOpzioniThread.join();

    return 0;
}

void controlLoop()
{
    cout << "Starting control loop" << endl;

    while (isRunning)
    {
        /* Controlla se il controllo è attivo */
        if (!controlLoopActive)
        {
            velocity[0] = 0;
            robot->move_lin_vel_wrf(velocity);
            delayDuration = SAMPLING_TIME_MICROS;
            while (!controlLoopActive)
                std::this_thread::sleep_for(std::chrono::microseconds(delayDuration));
        }
        start = getCurrentTimeMicros();

        /* Misura la distanza attuale tra sensore e ostacolo */
        currentDistance = -infraredSensor->getDistanceInMillimeters();
        startingReferenceDistance = currentDistance;

        /* Ostacolo fuori portata del sensore */
        if (currentDistance < -200)
        {
            handleOutOfRange();
        }

        /* Interpolazione del riferimento se necessario */
        if (interpolationActive)
        {
            interpolateReference();
        }

        /* Calcolo dell'errore e della velocità da comandare */
        error = currentReferenceDistance - currentDistance;
        output = regolatore->calculate_output(error);

        /* Controllo delle posizioni limite ammesse */
        if (robot->get_position() >= robot->POS_LIMIT_SUP)
        {
            // se la velocità è positiva resta fermo
            if (output > 0)
            {
                output = 0;
            }
        }
        else if (robot->get_position() <= robot->POS_LIMIT_INF)
        {
            // se la velocità è negativa resta fermo
            if (output < 0)
            {
                output = 0;
            }
        }

        /* Invia la velocità calcolata al Meca500 */
        velocity[0] = output;
        robot->move_lin_vel_wrf(velocity);

        /* Scrivi dati di controllo sul file csv */
        // "time,reference,position,measured_distance,error,velocity_control"
        writeDataToCsv(current_time, currentReferenceDistance, robot->get_position(), currentDistance, error, output, *csvLogger);

        /* Aspetta il tempo di campionamento corretto */
        delayDuration = SAMPLING_TIME_MICROS - (getCurrentTimeMicros() - start);
        std::this_thread::sleep_for(std::chrono::microseconds(delayDuration));

        // Incremento il tempo attuale
        current_time += SAMPLING_TIME;
    }
}

void handleOutOfRange()
{
    cout << "Sensor out of range.. stopping robot\n";
    cout << "Waiting for obstacle to be in range..\n";

    /* Ferma il Meca500 */
    velocity[0] = 0;
    regolatore->reset();
    robot->move_lin_vel_wrf(velocity);

    /* Aspetta che l'ostacolo torni all'interno della portata del sensore */
    while (currentDistance < -200)
    {
        start = getCurrentTimeMicros();
        currentDistance = -infraredSensor->getDistanceInMillimeters();

        /* Scrivi i dati di controllo sul file csv */
        writeDataToCsv(current_time, currentReferenceDistance, robot->get_position(), currentDistance, finalReferenceDistance - currentDistance, 0, *csvLogger);

        /* Aspetta il tempo di campionamento corretto */
        delayDuration = SAMPLING_TIME * 1e6 - (getCurrentTimeMicros() - start);
        std::this_thread::sleep_for(std::chrono::microseconds(delayDuration));
        current_time += SAMPLING_TIME;
    }

    cout << "Obstacle in range.. resuming control\n";

    /* L'ostacolo è tornato all'interno della portata del sensore */
    // È necessaria un'altra interpolazione
    interpolationActive = true;
}

void interpolateReference()
{
    // Inizia l'interpolazione a partire dalla distanza attuale del robot
    startingReferenceDistance = currentDistance;
    // Calcola la pendenza della retta interpolante
    interpolationSlope = (finalReferenceDistance - startingReferenceDistance) / interpolationDuration;
    // Inizializza il tempo di interpolazione all'istante attuale
    interpolationTime = current_time;

    // Calcola l'interpolazione
    currentReferenceDistance = interpolationSlope * (current_time - interpolationTime) + startingReferenceDistance;
    // Se la retta interpolata ha raggiunto o superato il valore finale termina l'interpolazione
    if ((interpolationSlope > 0 && currentReferenceDistance >= finalReferenceDistance) || (interpolationSlope <= 0 && currentReferenceDistance <= finalReferenceDistance))
    {
        currentReferenceDistance = finalReferenceDistance;
        interpolationActive = false;
    }
}

// Funzione che riceve continuamente i comandi dall'utente per eseguirli
void receiveCommands()
{
    string input;
    // Pulisci l'ingresso
    cin.clear();
    fflush(stdin);

    while (isRunning)
    {
        cout << "Inserisci comandi da eseguire --commandname=commandvalue [--help]: " << endl;
        getline(std::cin, input);

        // Esegui il parsing del comando ricevuto
        vector<string> tokens = splitString(input);

        int argc = tokens.size();
        char **argv = new char *[argc];
        for (int i = 0; i < argc; ++i)
        {
            argv[i] = new char[tokens[i].size() + 1];
            std::strcpy(argv[i], tokens[i].c_str());
        }

        // Esegui le istruzioni ricevute
        executeOptions(parseOptionTokens(argc, argv));
    }
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
    float pole_1 = 0.6;
    float zero_1 = 0.7967;
    float gain = 1.6334;
    vector<float> input_coeff{gain, -gain * zero_1};
    vector<float> output_coeff{2 * pole_1, -pole_1 * pole_1};
    regolatore = new Regolatore(output_coeff, input_coeff);
}

void setupCsvLogger()
{
    csvLogger = new CsvLogger(csvDataPath.c_str());
    csvLogger->write("time,reference,position,measured_distance,error,velocity_control\n");
}

void writeDataToCsv(float time, float reference, float position, float measured_distance, float error, float velocity_control, CsvLogger &logger)
{
    logger << current_time;
    logger << currentReferenceDistance;
    logger << robot->get_position();
    logger << currentDistance;
    logger << error;
    logger << output;
    logger.end_row();
}

void setupCommandHandlers()
{
    optionHandlers[HELP_COMMAND] = OptionHandler(handleHelp, helpMessage.str());
    optionHandlers[REFERENCE_COMMAND] = OptionHandler(handleRef, refMessage.str());
    optionHandlers[STOP_COMMAND] = OptionHandler(handleStop, stopMessage.str());
    optionHandlers[CALIBRATION_CURVE_COMMAND] = OptionHandler(handleCalibration, calMessage.str());
    optionHandlers[PAUSE_COMMAND] = OptionHandler(handlePause, pauseMessage.str());
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
    float vel[] = {0, 0, 0, 0, 0, 0};
    robot->move_lin_vel_wrf(vel);
    robot->deactivate();
    isRunning = false;
    controlLoopActive = true;
    return "Program stopped successfully";
}

string handlePause(string value)
{
    cout << "Pausing control\n";
    if (controlLoopActive)
    {
        cout << "Pausing control loop type --pause again to resume" << endl;
    }
    else
    {
        cout << "Resuming control loop..." << endl;
    }
    controlLoopActive = !controlLoopActive;
    return "";
}

string handleRef(string value)
{
    stringstream optionMessage;
    optionMessage << left << setw(message_length) << "Riferimento impostato a: " << value << "\n";
    finalReferenceDistance = -stof(value);
    interpolationActive = true;
    return optionMessage.str();
}

string handleCalibration(string value)
{
    stringstream optionMessage;
    vector<float> calibration_values = parseStringToVector(value);

    if (infraredSensor != nullptr)
    {
        infraredSensor->useCalibrationCurve(calibration_values[0], calibration_values[1]);
    }
    optionMessage << left << setw(message_length) << "Parametri calibrazione sensore: " << value << "\n";

    return optionMessage.str();
}

void moveRobotToPosition(vector<float> robot_position)
{
    robot->move_pose(
        robot_position[0],
        robot_position[1],
        robot_position[2],
        robot_position[3],
        robot_position[4],
        robot_position[5]);
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
        << "Ferma immediamente l'esecuzione del programma" << endl;
    pauseMessage
        << left
        << "  --" << setw(optionWidth) << PAUSE_COMMAND << setw(descriptionWidth)
        << "Interrompe l'esecuzione del ciclo di controllo o permette di farlo ripartire" << endl;
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

    for (int i = 0; i < argc; ++i)
    {
        string arg = argv[i];
        cout << "Comando ricevuto: " << argv[i] << endl;
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

vector<float> parseStringToVector(string input)
{
    vector<float> result;

    // Check if the string starts with a curly brace
    if (input.empty() || input[0] != '{')
    {
        cerr << "Error: Input string does not start with a curly brace.\n";
        return result;
    }

    stringstream ss(input);

    // Remove curly braces from the input string
    char discard;
    ss >> discard; // Discard the opening curly brace

    // Read the values into the vector
    float value;
    while (ss >> value)
    {
        result.push_back(value);

        // Check for comma and discard if present
        if (ss.peek() == ',')
            ss.ignore();
    }
    return result;
}

uint64_t getCurrentTimeMicros()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}