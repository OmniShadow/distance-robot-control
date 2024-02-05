
#include "meca500_ethercat_cpp/Robot.hpp"
#include "csvlogger/CsvLogger.hpp"
#include <pigpio.h>
#include <unistd.h>

int main(int argc, char *argv[])
{
    if (gpioInitialise() < 0)
        exit(1);
        
    /*robot, setup*/
    Robot robot(40, 240, 5000, "eth0", 0.0, 10);
    robot.reset_error();
    // robot.main();

    robot.set_conf(1, 1, -1);
    robot.move_pose(130, -170, 120, 90, 90, 0);
    robot.print_pose();

    /*files to write, setup*/
    CsvLogger input_logger("control/velocity_control");
    input_logger.write("time, value\n");

    CsvLogger output_velocity_logger("control/velocity_response");
    output_velocity_logger.write("time, value\n");

    CsvLogger output_position_logger("control/position_response");
    output_position_logger.write("time, value\n");

    /*squarewave generation, setup*/
    float input_velocity_mms = 10; // 10mm/s    (amplitude)
    double period_s = 6;           // 6sec      (period)
    float velocity[6] = {0, 0, 0, 0, 0, 0};
    /*squarewawe T = 6s, A=10mm/s*/

    /*time variables setup*/
    uint32_t startTime, currentTime, t0;
    t0 = gpioTick(); // time to start analysing response

    /*process*/
    while (true)
    {
        velocity[0] = input_velocity_mms;
        // input

        startTime = gpioTick();
        currentTime = gpioTick();
        while (currentTime - startTime >= period_s * 1e6 / 2) // Run the loop for 3 seconds
        {

            robot.move_lin_vel_wrf(velocity); // give 10mm/s or -10mm/s

            // Check if 3 secone passed. Exit cycle if true
            currentTime = gpioTick();

            // get datas
            input_logger << (currentTime - t0) / 1e6;
            input_logger << input_velocity_mms;
            input_logger.end_row();

            output_velocity_logger << (currentTime - t0) / 1e6;
            output_velocity_logger << robot.get_velocity();
            output_velocity_logger.end_row();

            output_position_logger << (currentTime - t0) / 1e6;
            output_position_logger << robot.get_position();
            output_position_logger.end_row();

            // short delay:
            usleep(1000);
        }

        input_velocity_mms = -input_velocity_mms;
    }
}
