import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys, os
from plot_utils import get_cov_ellipse

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
            state_belief_lines = [line for line in content if "target belief state:" in line]
            cov_belief_lines = [line for line in content if "target belief covariance:" in line]


            # Parse lines specific to each item
            measurements = [float(meas.split("Measurement:")[1]) for meas in measurement_lines]
            target_states = [np.fromstring(target.split("target:")[1].strip(), dtype=float, sep=',') for target in target_lines]
            ownship_states = [np.fromstring(ownship.split("ownship:")[1].strip(), dtype=float, sep=',') for ownship in ownship_lines]
            target_belief_states = [np.fromstring(belief.split("target belief state:")[1].strip(), dtype=float, sep=',') for belief in state_belief_lines]
            cov_belief_states = [np.fromstring(cov.split("target belief covariance:")[1].strip(), dtype=float, sep=',') for cov in cov_belief_lines]

            # print("Number of measurements: ", len(measurements))
            # print("Number of target states: ", len(target_states))
            # print("Number of ownship states: ", len(ownship_states))
            # print("Number of target belief states: ", len(target_belief_states))
            # print("Number of covariance states: ", len(cov_belief_states))

            return measurements, target_states, ownship_states, target_belief_states, cov_belief_states

    except FileNotFoundError:
        print(f"Error: The file '{log_file}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == '__main__':
    logdir = '../data/'
    logfile = 'output_log_UKF_20_1.txt'
    video_filename = "../data/UKF_20_1.gif"
    fps=100
    full_file = os.path.join(logdir, logfile)
    measurements, target_states, ownship_states, target_belief_states, cov_belief_states = read_log(full_file)
    print("Mean: ", np.mean(measurements)*180./np.pi, "Std: ", np.std(measurements)*180./np.pi)
    total_frames = len(measurements)
    vid_dir = '../data/'
    vid_name = 'animation'

    # Artist animation
    fig, ax = plt.subplots()
    bearing_points = np.linspace(0,100, 100)
    ownship_point = ax.plot(ownship_states[0][0], ownship_states[0][1], 'b.', label='sensor')[0]
    target_point = ax.plot(target_states[0][0], target_states[0][1], 'g.', label='Target')[0]
    bearing_line = ax.plot(ownship_states[0][0] + bearing_points*np.cos(measurements[0]), ownship_states[0][1] + bearing_points*np.sin(measurements[0]), 'g--', label='Measurement')[0]
    target_belief_point = ax.plot(target_belief_states[0][0], target_belief_states[0][1], 'rx', label='Target estimate')[0]
    ellipse_init = get_cov_ellipse(target_belief_states[0], cov_belief_states[0])
    cov_ellipse = ax.add_patch(ellipse_init.get_ellipse())
    ax.set(xlim=[-1, 11], ylim=[-1, 40], xlabel='x', ylabel='y')
    ax.legend(loc='upper left')

    def update(frame):
        # for each frame, update the data stored on each artist.
        x_own = ownship_states[frame][0]
        y_own = ownship_states[frame][1]
        x_tar = target_states[frame][0]
        y_tar = target_states[frame][1]
        tar_belief = target_belief_states[frame]
        cov = cov_belief_states[frame]
        ellipse = get_cov_ellipse(tar_belief, cov)

        ownship_point.set_xdata([x_own])
        ownship_point.set_ydata([y_own])
        target_point.set_xdata([x_tar])
        target_point.set_ydata([y_tar])
        bearing_line.set_xdata(x_own + bearing_points*np.cos(measurements[frame]))
        bearing_line.set_ydata(y_own + bearing_points*np.sin(measurements[frame]))
        target_belief_point.set_xdata([tar_belief[0]])
        target_belief_point.set_ydata([tar_belief[1]])

        cov_ellipse.set(center=(ellipse.center_x, ellipse.center_y), width=ellipse.width, height=ellipse.height,
                        angle=ellipse.angle, facecolor=ellipse.facecolor, edgecolor=ellipse.edgecolor)

        return ownship_point, target_point, bearing_line, target_belief_point, cov_ellipse

    ani = animation.FuncAnimation(fig=fig, func=update, frames=total_frames, interval=fps)
    ani.save(filename=video_filename, writer="pillow")
    # plt.show()