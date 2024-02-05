# PURPOSE: this script plot the response

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


def remove_trailing_commas(file_path):
    """
    Remove the final line commas in csv file
    
    Parameters:
    - arr: filepath of csv

    """

    # Open the input file in read mode
    with open(file_path, "r") as input_file:
        # Read lines from the input file
        lines = input_file.readlines()

    # Open the output file in write mode
    with open(file_path, "w") as output_file:
        # Iterate through each line and remove the last comma if it exists
        for line in lines:
            # Remove trailing whitespace and check if the last character is a comma
            if line.rstrip().endswith(","):
                # Remove the last comma and write the updated line to the output file
                # Remove last character (comma) and add newline
                updated_line = line.rstrip()[:-1] + "\n"
                output_file.write(updated_line)
            else:
                # If there is no trailing comma, write the line as it is to the output file
                output_file.write(line)

    
def generate_derivatives(folder_path):
    """
    generates from position_response and velocity_response the following cvs files:
    -velocity_response_computed
    -velocity_response_computed_filtered
    -velocity_response_filtered

    Parameters:
    - folder_path: path to folder "control"

    """

    #preparing csv files removing commas
    remove_trailing_commas(folder_path + "/position_response")
    remove_trailing_commas(folder_path + "/velocity_response")


    # 1 - GENERATING velocity_response_computed

    #start form position response
    file_path = folder_path + "/position_response"


    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(file_path)

    # Assuming the CSV file has two columns named 'time' and 'value'
    x_values = df['time']
    y_values = df['value']


    # Compute derivative
    dy_dx = np.gradient(y_values, x_values)


    # write to file csv velocity_response_computed
    file_to_write_path = folder_path + '/velocity_response_computed'

    # Write the data to the CSV file
    data = list(zip(x_values, dy_dx))

    with open(file_to_write_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        
        # Write header (optional)
        csv_writer.writerow(['time', 'value'])
        
        # Write the data rows
        csv_writer.writerows(data)


    # 2 - GENERATING velocity_response_computed_filtered
        
    # Compute filtering 
    window_size = 50  # Adjust the window size based on your data characteristics
    poly_order = 0
    filtered_derivative = savgol_filter(dy_dx, window_size, poly_order)

    # write to file csv velocity_response_computed
    file_to_write_path = folder_path + '/velocity_response_computed_filtered'

    # Write the data to the CSV file
    data = list(zip(x_values, filtered_derivative))

    with open(file_to_write_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        
        # Write header (optional)
        csv_writer.writerow(['time', 'value'])
        
        # Write the data rows
        csv_writer.writerows(data)



    # 3 - GENERATING velocity_response_filtered
        
    file_path = folder_path + "/velocity_response"

    # Read the CSV file into a pandas DataFrame
    df = pd.read_csv(file_path)

    # Assuming the CSV file has two columns named 'time' and 'value'
    x_values = df['time']
    y_values = df['value']

    # Compute filtering 
    window_size = 50  # Adjust the window size based on your data characteristics
    poly_order = 1
    filtered_derivative = savgol_filter(y_values, window_size, poly_order)

    # write to file csv velocity_response_computed
    file_to_write_path = folder_path + '/velocity_response_filtered'

    # Write the data to the CSV file
    data = list(zip(x_values, filtered_derivative))

    with open(file_to_write_path, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        
        # Write header (optional)
        csv_writer.writerow(['time', 'value'])
        
        # Write the data rows
        csv_writer.writerows(data)


def cut_spikes(arr, threshold, value):
    """
    Set elements of an array above (module) a threshold to value.
    
    Parameters:
    - arr: NumPy array
    - threshold: Threshold value
    - value: substitute value
    
    Returns:
    - Modified array
    """
    arr[arr > threshold] = value
    arr[arr < -threshold] = value

    return arr

def graph_plots(folder_path):

    #takes all filenames
    file_list = os.listdir(folder_path)

    # Remove "resume_plot.png" from the array
    if "resume_plot.png" in file_list:
        file_list.remove("resume_plot.png")

    #analize file by file 
    for file_name in file_list:
        # Construct the full path to the file
        file_path = folder_path + "/" + file_name

        # Remove final commas of the datas
        remove_trailing_commas(file_path)
        
        # Read the CSV file into a pandas DataFrame
        df = pd.read_csv(file_path)

        # Assuming the CSV file has two columns named 'time' and 'value'
        x_values = df['time']

        # Extract the first N elements (to let the array have the same lenght)
        N = 3500
        x_values = x_values[:N]

        if file_name == "position_response":
            position_response_values = df['value']
            position_response_values = position_response_values[:N]
        
        if file_name == "velocity_control":
            velocity_control_values = df['value']
            velocity_control_values = velocity_control_values[:N]
        
        if file_name == "velocity_response":
            velocity_response_values = df['value']
            velocity_response_values = velocity_response_values[:N]
            velocity_response_values = cut_spikes(velocity_response_values, 30, 30)
        
        if file_name == "velocity_response_filtered":
            velocity_response_filtered_values = df['value']
            velocity_response_filtered_values = velocity_response_filtered_values[:N]
            velocity_response_filtered_values = cut_spikes(velocity_response_filtered_values, 30, 30)
        
        if file_name == "velocity_response_computed":
            velocity_response_computed_values = df['value']
            velocity_response_computed_values = velocity_response_computed_values[:N]
            velocity_response_computed_values = cut_spikes(velocity_response_computed_values, 30, 30)
        
        if file_name == "velocity_response_computed_filtered":
            velocity_response_computed_filtered_values = df['value']
            velocity_response_computed_filtered_values = velocity_response_computed_filtered_values[:N]
        
    # Create a 3x2 grid of subplots
    fig, axes = plt.subplots(nrows=3, ncols=2, figsize=(12, 8))

    # Plot data on each subplot
    axes[0, 0].plot(x_values, velocity_control_values)
    axes[0, 0].set_title('Velocity control (input)')
    axes[0, 0].set_xlabel('time[s]')
    axes[0, 0].set_ylabel('velocity[mm/s]')

    axes[0, 1].plot(x_values, position_response_values)
    axes[0, 1].set_title('Meca position response')
    axes[0, 1].set_xlabel('time[s]')
    axes[0, 1].set_ylabel('position[mm]')

    axes[1, 0].plot(x_values, velocity_response_values)
    axes[1, 0].set_title('Meca velocity response')
    axes[1, 0].set_xlabel('time[s]')
    axes[1, 0].set_ylabel('velocity[mm/s]')

    axes[1, 1].plot(x_values, velocity_response_computed_values)
    axes[1, 1].set_title('Computed velocity response')
    axes[1, 1].set_xlabel('time[s]')
    axes[1, 1].set_ylabel('velocity[mm/s]')

    axes[2, 0].plot(x_values, velocity_response_filtered_values)
    axes[2, 0].set_title('Filtered Meca velocity response')
    axes[2, 0].set_xlabel('time[s]')
    axes[2, 0].set_ylabel('velocity[mm/s]')

    axes[2, 1].plot(x_values, velocity_response_computed_filtered_values)
    axes[2, 1].set_title('Filtered computed velocity response')
    axes[2, 1].set_xlabel('time[s]')
    axes[2, 1].set_ylabel('velocity[mm/s]')

    # Adjust layout to prevent clipping of titles
    plt.tight_layout()

    #save the plot
    plt.savefig(folder_path + '/resume_plot.png')

    # Show the plots
    plt.show()



def main():
    
    # Check if a folder path is provided as a command-line argument
    if len(sys.argv) != 2:
        print("Usage: python script.py /folder_to_analyse")
        sys.exit(1)
    
    # Get the folder path from the command-line argument
    folder_path = sys.argv[1]

    # Compute
    generate_derivatives(folder_path)
    graph_plots(folder_path)




if __name__ == "__main__":
    main()