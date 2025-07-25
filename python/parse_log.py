import numpy as np
import matplotlib.pyplot as plt
# import matplotlib

# matplotlib.use('TkAgg')
import matplotlib.animation as animation
# from matplotlib.backends.backend_agg import FigureCanvasAgg
# import imageio
import sys, os

plt.rcParams["axes.grid"] = True

# Optional: Customize grid appearance (e.g., color, linestyle, linewidth)
plt.rcParams["grid.color"] = "gray"
plt.rcParams["grid.linestyle"] = "--"
plt.rcParams["grid.linewidth"] = 0.5


def read_log(log_file):

    try:
        with open(log_file, 'r') as file:

            content = file.readlines()

            # Separate out lines for each item
            measurement_lines = [line for line in content if "Measurement:" in line]
            target_lines = [line for line in content if "target:" in line]
            ownship_lines = [line for line in content if "ownship:" in line]


            # Parse lines specific to each item
            measurements = [float(meas.split("Measurement:")[1]) for meas in measurement_lines]
            target_states = [np.fromstring(target.split("target:")[1].strip(), dtype=float, sep=',') for target in target_lines]
            ownship_states = [np.fromstring(ownship.split("ownship:")[1].strip(), dtype=float, sep=',') for ownship in ownship_lines]

            print("Number of measurements: ", len(measurements))
            print("Number of target states: ", len(target_states))
            print("Number of ownship states: ", len(ownship_states))

            return measurements, target_states, ownship_states

    except FileNotFoundError:
        print(f"Error: The file '{log_file}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == '__main__':
    logdir = '../data/'
    logfile = 'output_log.txt'

    full_file = os.path.join(logdir, logfile)

    measurements, target_states, ownship_states = read_log(full_file)
    print("Mean: ", np.mean(measurements)*180./np.pi, "Std: ", np.std(measurements)*180./np.pi)
    fps=1000
    vid_dir = '../data/'
    vid_name = 'animation'

    # Artist animation
    # fig, ax = plt.subplots(figsize=(8,8))
    # images = []
    # for i in range(len(measurements)):
    #     fig.clf()
    #     ax = fig.add_subplot()
    #     ax.set_xlim(0, 11)
    #     ax.set_ylim(0, 2)
    #     ax.plot(ownship_states[i][0], ownship_states[i][1], 'b.', label='Sensor')
    #     container = ax.plot(target_states[i][0], target_states[i][1], 'rx', label='Target')
    #     images.append(container)
    #
    #
    # ani = animation.ArtistAnimation(fig=fig, artists=images, interval=fps)
    # plt.show()
    # ani.save(filename="../data/pillow_example.gif", writer="pillow")

    # Artist animation
    fig, ax = plt.subplots()
    bearing_points = np.linspace(0,20, 100)
    ownship_point = ax.plot(ownship_states[0][0], ownship_states[0][1], 'b.', label='sensor')[0]
    target_point = ax.plot(target_states[0][0], target_states[0][1], 'rx', label='Target')[0]
    bearing_line = ax.plot(ownship_states[0][0] + bearing_points*np.cos(measurements[0]), ownship_states[0][1] + bearing_points*np.sin(measurements[0]), 'g--', label='Measurement')[0]
    ax.set(xlim=[-1, 11], ylim=[-1, 2], xlabel='x', ylabel='y')
    ax.legend()


    def update(frame):
        # for each frame, update the data stored on each artist.
        x_own = ownship_states[frame][0]
        y_own = ownship_states[frame][1]
        x_tar = target_states[frame][0]
        y_tar = target_states[frame][1]
        # update the scatter plot:
        # data = np.stack([x, y]).T
        # scat.set_offsets(data)
        # update the line plot:
        ownship_point.set_xdata([x_own])
        ownship_point.set_ydata([y_own])
        target_point.set_xdata([x_tar])
        target_point.set_ydata([y_tar])
        bearing_line.set_xdata(x_own + bearing_points*np.cos(measurements[frame]))
        bearing_line.set_ydata(y_own + bearing_points*np.sin(measurements[frame]))
        return ownship_point, target_point, bearing_line

    ani = animation.FuncAnimation(fig=fig, func=update, frames=10, interval=fps)
    ani.save(filename="../data/pillow_example.gif", writer="pillow")
    # plt.show()