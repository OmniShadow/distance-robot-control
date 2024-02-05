#ifndef ROBOT_H
#define ROBOT_H

#include <cstdint>
#include <iostream>
#include <iostream>
#include "Master.h"
// #include "Meca500.h"
#include "Controller.h"
#include <functional>
#include <stdexcept>
#include <chrono>
#include <fstream>
#include <cmath>

// void get_joints_vel_with_jacobian(double velocity, float *joints, float *joints_vel);

class Robot
{
private:
    sun::Meca500 meca500;
    sun::Master master;
    sun::Controller controller;
    bool as, hs, sm, es, pm, eob, eom;
    float joint_angles[6];
    float joints[6] = {0, 0, 0, 0, 60, 0};
    float omega[6] = {0, 0, 0, 0, 0, -30};
    float position[6] = {0, 0, 0, 0, 0, 0};
    double last_pos = 0;
    double last_vel = 0;

    double costante_tempo_filtro = 10e-3;

    int activateRob, deactivateRob, homeRob;
    const uint32_t TARGET_CYCLE_TIME_MICROSECONDS;
    char network_interface[50];
    static void update_data();
    bool block_ended();
    bool movement_ended();

public:
    const double POS_LIMIT_INF;
    const double POS_LIMIT_SUP;
    /*CONSTRUCTORS*/
    Robot(double pos_limit_inf,
          double pos_limit_sup,
          uint32_t target_cycle_time_microseconds,
          char *network_interface_in,
          float blending_percentage,
          float cart_accel_limit);
    ~Robot();

    /*METHODS*/
    void deactivate();
    void reset_error();
    void set_conf(short c1, short c2, short c3);

    double get_position();
    void get_pose(float *x);
    void print_pose();
    double get_velocity();

    void move_lin_vel_trf(float velocity[6]);
    void move_lin_vel_trf_x(double velocity);
    void move_lin_rel_trf(double x, double y, double z, double alpha, double beta, double gamma);
    void move_joints_vel(float *w);
    void get_joints(float *joints);
    void move_pose(double x, double y, double z, double alpha, double beta, double gamma);
    void move_lin(double x, double y, double z, double alpha, double beta, double gamma);
    void move_lin_rel_wrf(double x, double y, double z, double alpha, double beta, double gamma);
    void move_lin_vel_wrf(float velocity[6]);
};

#endif