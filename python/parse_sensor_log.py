import numpy as np
import matplotlib.pyplot as plt
import sys, os


def read_log(log_file):

    try:
        with open(log_file, 'r') as file:
            content = file.readlines()
            measurements = [float(meas.split("Measurement:")[1]) for meas in content]
            print("Number of measurements: ", len(measurements))
            return measurements
    except FileNotFoundError:
        print(f"Error: The file '{log_file}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == '__main__':
    logdir = '../data/'
    logfile = 'output_log.txt'

    full_file = os.path.join(logdir, logfile)

    measurements = read_log(full_file)
    print("Mean: ", np.mean(measurements)*180./np.pi, "Std: ", np.std(measurements)*180./np.pi)
    fig, ax = plt.subplots()
    ax.hist(measurements, label='Measurements', bins=100)
    ax.set_xlabel('Angle (radians)')
    ax.axvline(x=np.pi/2, color = 'red', linestyle = '--', label='Pi/2')
    ax.set_ylabel('Count')
    ax.legend()
    plt.show()

