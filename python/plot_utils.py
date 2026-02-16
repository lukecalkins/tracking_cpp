import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse


class CovEllipse:

    def __init__(self, center_x, center_y, height, width, angle, facecolor='none', edgecolor='red'):
        self.center_x = center_x
        self.center_y = center_y
        self.height = height
        self.width = width
        self.angle = angle
        self.facecolor = facecolor
        self.edgecolor = edgecolor
        self.ellipse = Ellipse((self.center_x, self.center_y), width=self.width, height=self.height,
                          angle=self.angle, facecolor='none', edgecolor='red', label=r'1$\sigma$ covariance')

    def get_ellipse(self):
        return self.ellipse
def get_cov_ellipse(targ_belief_state, cov_belief_state, n_std=2):

    # reshape 16 element array to 4x4
    cov_mat = cov_belief_state.reshape(4, 4, order='C')
    pos_cov = cov_mat[:2, :2]
    # print(pos_cov)

    lam, V = np.linalg.eig(pos_cov)
    # print("lam 1: ", lam[0], "lam 2: ", lam[1])
    width = 1 * np.sqrt(lam[0]) # Major axis length
    height = 1 * np.sqrt(lam[1]) # Minor axis length
    D = np.array([[lam[0], 0],[0, lam[1]]])
    # print(D)
    # print(np.matmul(pos_cov, V), "\n?=\n", np.matmul(D, V))
    angle = np.degrees(np.arctan2(V[1, 0], V[0, 0]))
    # print("angle: ", angle, " degrees")
    center_x = targ_belief_state[0]
    center_y = targ_belief_state[1]
    ellipse = CovEllipse(center_x, center_y, height, width, angle)

    return ellipse
