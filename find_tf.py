# PURPOSE: this script find the transfer function of the system using open loop calibration technique
# in particular we found T and Tau


import os  # interation with operating system
import sys  # interation with operating system
import pandas as pd  # draw graph
import matplotlib.pyplot as plt  # draw graph
# import re  # support for working with regular expression (string)
# import numpy as np  # support for working with multidimension variables (array)
# from scipy.stats import norm  # probability and statistic
# from sklearn.linear_model import LinearRegression  # for best fit
import numpy as np
from scipy.signal import savgol_filter, butter, filtfilt
import csv


def rise_index(array):
    
    """
    find the first index where array is different from the beginning
    
    Parameters:
    - array: array to work on

    """

    # Store the first element as the initial value to compare
    # Prepare to find index of rise
    initial_value = array[0]
    index_of_rise = -1

    # Iterate through the vector starting from the second element
    for index, value in enumerate(array[1:], start=1):
        if value != initial_value:
            # Found a different value
            index_of_rise = index
            break
    
    return index_of_rise


def find_Tau(folder_path, file_name):

    """
    find the delay of the signal stored in folder_path/file_name
    return Tau or -1 if signal is constant

    """

    file_path = folder_path + "/" + file_name

    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(file_path)

    # Assuming the CSV file has two columns named 'time' and 'value'
    x_values = df['time']
    y_values = df['value']

    # Rounding to zero the infinitesimal value (|| < 1e-4 m)
    for index, value in enumerate(y_values[0:], start=0):
        if abs(value) < 1e-4:
            y_values[index] = round(abs(value))

    # Compute index of rise
    index_of_rise = rise_index(y_values)
    

    # Computed delay
    if index_of_rise != -1:
        return x_values[index_of_rise]
    else:
        return -1   #signal is constant

    
def find_T(folder_path):

    
    """
    find T of the signal stored in folder_path/velocity_response_computed
    return T or -1 if some mistake

    """

    # Get the file path where take data
    file_path = folder_path + "/velocity_response_computed_filtered"

    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(file_path)

    # Assuming the CSV file has two columns named 'time' and 'value'
    x_values = df['time']
    y_values = df['value']

    # Extract the first N elements (indicial response)
    N = 500
    x_values = x_values[:N]
    y_values = y_values[:N]


    
    #finding delay Tau_temp in this signal 
    Tau_temp = find_Tau(folder_path, "velocity_response_computed_filtered")

    # Compute derivative
    dy_dx = np.gradient(y_values, x_values)

    # Find oblique tangent flex y = m(x-xo) + yo
    m = max(dy_dx)                                              #slope m
    index_to_max = np.where(dy_dx == m)[0][0] + 1
    time_to_max = x_values[index_to_max]                        #xo
    value_in_max = y_values[index_to_max]                       #yo
    q = - m * time_to_max + value_in_max                        #intercept q
    #note: that +1 is to fix the advance of the derivative effect

    #Line is: y=mx+q
    y_flex = m * x_values + q
    y_flex = y_flex[20:50]
    x_flex = x_values[20:50]

    #regime value is indicated in the foldername (last two carachters)
    regime_value = folder_path[-2:]
    #print(regime_value)
    y_regime = 0 * x_values + int(regime_value)

    #Create a two-frame subplots
    fig, axes = plt.subplots(nrows=1, ncols=2, figsize=(12, 4))

    # Plot data on each subplot
    axes[0].plot(x_values, y_values)
    axes[0].plot(x_flex, y_flex, color='red')
    axes[0].plot(x_values, y_regime, color='green')
    axes[0].set_title('velocity')
    axes[0].set_xlabel('time[s]')
    axes[0].set_ylabel('velocity[mm/s]')

    axes[1].plot(x_values, dy_dx)
    axes[1].set_title('derivative')
    axes[1].set_xlabel('time[s]')
    axes[1].set_ylabel('position[mm]')

    # Adjust layout to prevent clipping of titles
    plt.tight_layout()

    #save the plot
    plt.savefig(folder_path + '/open_loop_calibration.png')
    
    # Show the plots
    plt.show()



    #find T solving y_regime = m(T+Tau_temp) + q
    T = (int(regime_value) - q) / m - Tau_temp

    return T


def main():
    
    # Check if a folder path is provided as a command-line argument
    if len(sys.argv) != 2:
        print("Usage: python script.py /folder_to_analyse")
        sys.exit(1)
    # Get the folder path from the command-line argument
    folder_path = sys.argv[1]


    # Compute
    Tau = find_Tau(folder_path, "position_response")
    T = find_T(folder_path)

    # Show output
    print("Tau: " + str(Tau) + " T: "+ str(T))
    


if __name__ == "__main__":
    main()



