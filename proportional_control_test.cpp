#include "distance_sensor/include/DistanceSensor.hpp"
#include "distance_sensor/include/UltrasonicSensor.hpp"
#include "distance_sensor/include/InfraredSensor.hpp"
#include "meca500_ethercat_cpp/Robot.hpp"

using namespace std;

int main(int argc, char *argv[])
{
    InfraredSensor sensor(InfraredSensor::USER_INPUT);
    Robot robot(100, 200, 5000, "eth0", 0.0, 10);
    robot.reset_error();
    // robot.main();
    robot.set_conf(1, 1, -1);
    robot.move_pose(130, -170, 120, 90, 90, 0);
    robot.print_pose();
    float velocity[6] = {0, 0, 0, 0, 0, 0};
    float position[6];

    float distance;
    float vel = 15;
    int pos = 0;
    float setpoint = 50;
    float kp = 1;
    while (true)
    {
        robot.get_pose(position);
        distance = sensor.getDistanceInMillimeters();
        cout << "Measured distance: " << distance << endl;
        cout << "Position: " << position[0] << endl;
        if (distance < 200)
        {

            vel = -kp * (setpoint - distance);
            cout << "Velocity: " << vel << endl;
            if ((vel < 0 && position[0] <= robot.POS_LIMIT_INF) ||
                (vel > 0 && position[0] >= robot.POS_LIMIT_SUP))
                vel = 0;
            velocity[pos] = vel;

            robot.move_lin_vel_wrf(velocity);
        }
        else
        {
            velocity[pos] = 0;
            robot.move_lin_vel_wrf(velocity);
        }
    }
}
