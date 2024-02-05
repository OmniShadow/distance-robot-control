#include "Robot.hpp"

int main(int argc, char *argv[])
{
    Robot robot(-30, 200, 5000, "eth0", 0.0, 100);
    robot.reset_error();
    // robot.main();
    robot.set_conf(1, 1, -1);
    robot.move_pose(130, -170, 120, 90, 90, 0);
    robot.print_pose();
    float velocity[6] = {0, 0, 0, 0, 0, 0};
    float vel = 10;
    int i = 0;
    while (true)
    {
        velocity[i%3] = vel;
        robot.move_lin_vel_wrf(velocity);
        usleep(2e+6);
        velocity[i%3] = -vel;
        robot.move_lin_vel_wrf(velocity);
        usleep(2e+6);
        velocity[i%3] = 0;
        i++;
    }
}
