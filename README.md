# distance-robot-control

In this folder u can access the tools for the project of the digital regulator and run it

## How to compile

If you are using cmake, after you've cloned the repository into your disposal, digit (by cmd) the command "make"


## How to find the velocity response and transfer function by Meca

If you desire to find the velocity response by Meca, execute velocity_response.
It will generate datas in a folder called "control...".

If u want to set the parameters of the velocity waveform, change the code in velocity_response.cpp and re-compile.

Then, execute the Python script "plot_response.py" passing the generated folder to plot the data in some graphs. It's needed to generate other useful csv files.

Then, only after executed the previous script, execute "find_tf.py" to find the parameters of Meca500 fdt.

You can test the founded fdt running the Matlab script "test_tf.mlx"


## How to project the digital regulator:

If you desire to project the digital regulator, open find_R, write your parameters and execute the script.


## How to run the close loop control:

If you desire to run close loop control, you need to place the obstacle opposite to the sensor and run regulator.

If u need to set the parameters for calibration, open the code, change them and re-compile.



