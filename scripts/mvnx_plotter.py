from mvnx_parser import MVNX

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np


class mvnx_plotter():
    def __init__(self):
        self.mvnx = MVNX()
        _, self.segments, self.sensors, self.joints, self.frames = self.mvnx.parse_all()

    def plot_joint(self, segment_name="Pelvis"):
        mpl.style.use('seaborn')  # Style can be changed to "normal"

        fig, ax = plt.subplots(figsize=(12, 3))
        ax.set_title(segment_name, color='C0')
        ax.set_xlabel("seconds (s)")
        ax.set_ylabel("Q(t)")
        x, y, z = [], [], []
        position = self.frames[0].PLOT_POSITIONS

        for frame in self.frames[1:]:
            position = np.vstack((position, frame.PLOT_POSITIONS))

        x, y, z = position[:, 0], position[:, 1], position[:, 2]

        ax.plot(x, label='P_x')
        ax.plot(y, label='P_y')
        ax.plot(z, label='P_z')
        ax.legend()
        plt.show()


if __name__ == "__main__":
    plotter = mvnx_plotter()
    plotter.plot_joint()
