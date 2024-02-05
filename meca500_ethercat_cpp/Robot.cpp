#include "Robot.hpp"
#include <iostream>
#include "Master.h"
// #include "Meca500.h"
#include "Controller.h"
#include <functional>
#include <stdexcept>
#include <chrono>
#include <fstream>
#include <cmath>
#include "joints_vel.h"

Robot::Robot(double pos_limit_inf, double pos_limit_sup, uint32_t target_cycle_time_microseconds,
             char *network_interface_in,
             float blending_percentage,
             float cart_accel_limit) : POS_LIMIT_INF(pos_limit_inf),
                                       POS_LIMIT_SUP(pos_limit_sup),
                                       master(network_interface_in, FALSE, EC_TIMEOUT_TO_SAFE_OP),
                                       meca500(1, &master),
                                       controller(&meca500, 0.5),
                                       TARGET_CYCLE_TIME_MICROSECONDS(target_cycle_time_microseconds)

{
    using namespace sun;
    using namespace std;

    sprintf(network_interface, network_interface_in);

    std::cout << "SOEM (Simple Open EtherCAT Master)\nStarting master...\n";

    uint16 state_check;

    long time_new_packet[500];

    master.setupSlave(meca500.getPosition(), Meca500::setup_static);

    master.configDC();
    master.configMap();

    meca500.assign_pointer_struct();

    master.movetoState(meca500.getPosition(), EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
    master.createThread(TARGET_CYCLE_TIME_MICROSECONDS * 1e+3);

    master.movetoState(meca500.getPosition(), EC_STATE_OPERATIONAL, EC_TIMEOUT_TO_SAFE_OP);

    meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
    printf("\nActivate: %d\n", as);
    printf("Homed: %d\n", hs);
    printf("Sim: %d\n", sm);
    printf("Error: %d\n", es);

    if (meca500.setBlending(blending_percentage) != 0)
    {
        cout << "Error set blending\n";
    }

    if (meca500.setCartAcc(cart_accel_limit) != 0)
    {
        cout << "error set cart acceleration\n";
    }

    if (meca500.setJoinAcc(150.0) != 0) // setting joints acc to max
    {
        cout << "error set joints acceleration\n";
    }

    meca500.setJoinVel(600);

    meca500.activateRobot();
    meca500.home();

    meca500.setPoint(1);
    last_pos = 0;
}

bool Robot::block_ended()
{
    meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
    std::cout << "eob:" << eob << std::endl;
    return eob;
}

bool Robot::movement_ended()
{
    meca500.getStatusRobot(as, hs, sm, es, pm, eob, eom);
    std::cout << "eom:" << eom << std::endl;
    return eom;
}

Robot::~Robot()
{
    master.close_master();
    master.stampa();
    master.waitThread();
}

void Robot::deactivate()
{
    meca500.deactivateRobot();
}

void Robot::reset_error()
{
    meca500.resetError();
}

void Robot::set_conf(short c1, short c2, short c3)
{
    float conf[] = {(float)c1, (float)c2, (float)c3};
    int n = meca500.setConf(conf);
    std::cout << n << std::endl;
}

double Robot::get_position() // Returning x position of the Robot (horizontal)
{
    float pose[6];
    meca500.getPose(pose);

    return pose[0];
}

void Robot::get_pose(float *x) // Returning pose, cartesian(mm) and angular(°)
{
    meca500.getPose(x);

    /*
    for (int i = 0; i < 3; i++)
    {
        x[i] *= 1e-3;                   //mm -> meters
    }

    for (int i = 3; i < 6; i++)
    {
        x[i] = x[i] * M_PI / 180.0;     //degres -> rad
    }
    */
}

double Robot::get_velocity()
{
    // implementato con derivatore discreto in banda sulla posizione

    double pos = get_position() * 1e-3; // mm to meters
    double T = TARGET_CYCLE_TIME_MICROSECONDS * 1e-6;
    double tau = costante_tempo_filtro;
    double vel = -(T - 2 * tau) / (T + 2 * tau) * last_vel + 2 / (T + 2 * tau) * pos - 2 / (T + 2 * tau) * last_pos;
    // vel = (pos-last_pos)/T;
    last_pos = pos;
    last_vel = vel;

    return vel * 1e3; // m/s -> mm/s
}

void Robot::print_pose()
{
    float pose[6];
    meca500.getPose(pose);
    for (int i = 0; i < 6; i++)
    {
        std::cout << i << ": " << pose[i] << std::endl;
    }
}

void Robot::move_lin_vel_wrf(float velocity[6]) // input is in mm/s, ranging from -1000 to 1000
{

    // vel[0] = (float)velocity * 1e+3;
    meca500.moveLinVelWRF(velocity);
}

void Robot::move_pose(double x, double y, double z, double alpha, double beta, double gamma)
{
    if (x >= POS_LIMIT_SUP || x <= POS_LIMIT_INF) // dangerous position
    {
        printf("impossibile to move: %d is a dangerous position:\n", x);
    }
    else
    {
        float pose[] = {(float)x, (float)y, (float)z, (float)alpha, (float)beta, (float)gamma};
        meca500.movePose(pose);
        usleep(0.2e+6);
        while (!movement_ended())
        {
            printf("waiting for robot to finish moving\n");
            usleep(1e+6);
        }
    }
}

void Robot::move_lin(double x, double y, double z, double alpha, double beta, double gamma)
{
    if (x >= POS_LIMIT_SUP || x <= POS_LIMIT_INF) // dangerous position
    {
        printf("impossibile to move: %d is a dangerous position:\n", x);
    }
    else
    {
        float pose[] = {(float)x, (float)y, (float)z, (float)alpha, (float)beta, (float)gamma};

        meca500.moveLin(pose);
    }
}

void Robot::move_lin_rel_wrf(double x, double y, double z, double alpha, double beta, double gamma)
{
    if (x >= POS_LIMIT_SUP || x <= POS_LIMIT_INF) // dangerous position
    {
        printf("impossibile to move: %d is a dangerous position:\n", x);
    }
    else
    {
        float pose[] = {(float)x, (float)y, (float)z, (float)alpha, (float)beta, (float)gamma};

        meca500.moveLinRelWRF(pose);
    }
}

// joints control

void Robot::move_joints_vel(float *w)
{
    for (int i = 0; i < 6; i++)
    {
        w[i] = (w[i] * 180.0) / M_PI;
    }
    meca500.moveJointsVel(w);
}

void Robot::get_joints(float *joints) // Returning Horizontal position of the Robot
{
    meca500.getJoints(joints);
    for (int i = 0; i < 6; i++)
    {
        joints[i] = (joints[i] * M_PI) / 180.0;
    }
}

// trf control

void Robot::move_lin_rel_trf(double x, double y, double z, double alpha, double beta, double gamma)
{
    float pose[] = {(float)x, (float)y, (float)z, (float)alpha, (float)beta, (float)gamma};

    meca500.moveLinRelTRF(pose);
}

void Robot::move_lin_vel_trf_x(double velocity) // input is in m/s, ranging from -1 to 1
{
    float joints[6];
    float joints_vel[6];
    float pose[6];
    if (velocity > 0 && get_position() > POS_LIMIT_SUP)
    {
        velocity = 0;
    }
    if (velocity < 0 && get_position() < POS_LIMIT_INF)
    {
        velocity = 0;
    }
    get_pose(pose);
    get_joints(joints);
    get_joints_vel_with_jacobian(velocity, joints, joints_vel, pose);
    move_joints_vel(joints_vel);
}

void Robot::move_lin_vel_trf(float velocity[6]) // input is in mm/s, ranging from -1000 to 1000
{
    // float vel[6] = {0, 0, 0, 0, 0, 0};
    // if (velocity > 0 && get_position() >= POS_LIMIT_SUP)
    // {
    //     velocity = 0;
    // }
    // if (velocity < 0 && get_position() <= POS_LIMIT_INF)
    // {
    //     velocity = 0;
    // }

    // vel[0] = (float)velocity;
    meca500.moveLinVelTRF(velocity);
}
