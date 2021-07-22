from os import remove
from mvnx_parser import MVNX
from utils import logger

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np


class mvnx_plotter():
    def __init__(self):
        self.mvnx = MVNX()
        _, self.segments, self.sensors, self.joints, self.frames = self.mvnx.parse_all()

    def plot_joint(self, ID=1):
        """Plot joint position of the preferred joint from the mvnx file

        Args:
            ID (INT, optional): The ID number joint segment from the MVNX manual. Defaults to "Pelvis".
        """
        segment_name = ""

        for segment in plotter.segments:
            if ID == int(segment.ID):
                segment_name = segment.NAME
                break

        print(logger.OKCYAN +
              "Plotting segment: {}".format(segment_name) + logger.ENDC)

        mpl.style.use('seaborn')  # Style can be changed to "normal"

        fig, ax = plt.subplots(figsize=(12, 3))
        ax.set_title("Position graph of joint segment: {}".format(
            segment_name), color='C0')
        ax.set_xlabel("seconds (s)")
        ax.set_ylabel("P(t)")

        x, y, z = [], [], []
        position = self.frames[0].PLOT_POSITIONS

        for frame in self.frames[1:]:
            position = np.vstack((position, frame.PLOT_POSITIONS))

        x, y, z = position[:, (ID-1) + 0], position[:,
                                                    (ID-1) + 1], position[:, (ID - 1) + 2]

        ax.plot(x, label='P_x')
        ax.plot(y, label='P_y')
        ax.plot(z, label='P_z')
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

    ID = 1
    segment_name = None

    plotter = mvnx_plotter()
    # plotter.plot_com()

    # To plot the required joint from the file, gather information from the segment class
    while ID is not 0:
        print(logger.OKBLUE + "-"*50 + logger.ENDC)
        print(logger.OKBLUE +
              "Select the preferred joint position from below: (0 to exit)" + logger.ENDC)

        for segment in plotter.segments:
            print(logger.OKBLUE + "{}. \t{}".format(segment.ID,
                  segment.NAME) + logger.ENDC)

        ID = int(input("> "))

        if ID != 0:
            # plotter.plot_joint(ID)
            plotter.plot_com()
