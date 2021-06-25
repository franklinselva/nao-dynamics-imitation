from mvnx_parser import MVNX

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np


class mvnx_plotter():
    def __init__(self):
        self.mvnx = MVNX()
        _, self.segments, self.sensors, self.joints, self.frames = self.mvnx.parse_all()

    def plot_joint(self, segment_name="Pelvis"):
        """Plot joint position of the preferred joint from the mvnx file

        Args:
            segment_name (str, optional): The name of the joint segment from the MVNX manual. Defaults to "Pelvis".
        """
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

        ax.plot(x, label='Q_x')
        ax.plot(y, label='Q_y')
        ax.plot(z, label='Q_z')
        ax.legend()
        plt.show()

    def plot_com(self):
        """Plot COM trajectory of the pelvis joint.
        """
        mpl.style.use('seaborn')  # Style can be changed to "normal"

        fig, ax = plt.subplots(figsize=(12, 3))
        ax.set_title("CoM Projection of Pelvis Joint", color='C0')
        ax.set_xlabel("seconds (s)")
        ax.set_ylabel("CoM")
        x, y, z = [], [], []
        position = self.frames[0].PLOT_CoM

        for frame in self.frames[1:]:
            position = np.vstack((position, frame.PLOT_CoM))

        x, y, z = position[:, 0], position[:, 1], position[:, 2]

        ax.plot(x, label='COM_x')
        ax.plot(y, label='COM_y')
        ax.plot(z, label='COM_z')
        ax.legend()
        plt.show()


if __name__ == "__main__":
    plotter = mvnx_plotter()
    plotter.plot_com()
    plotter.plot_joint()
