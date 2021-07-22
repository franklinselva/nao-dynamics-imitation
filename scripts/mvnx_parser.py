from posixpath import join
import numpy as np
import xml.etree.ElementTree as ET
import os

from utils import logger, print_segment, print_joint
from utils import FRAME_ID
from utils import joint as joint_holder
from utils import segment as segment_holder
from utils import frame as frame_holder

dirpath = os.path.abspath("../")


class MVNX:
    """
    The abstract parser object to run through the XML tree structure of the MVNX file format
    and extract the relevant information into dictionaries and numpy arrays. Super simple, needs
    refactoring at the moment.

    Can also be used as a command line tool.
    """

    def __init__(self, path=None, orientation=None, position=None, velocity=None,
                 acceleration=None, angularVelocity=None, angularAcceleration=None,
                 footContacts=None, sensorFreeAcceleration=None, sensorMagneticField=None,
                 sensorOrientation=None, jointAngle=None, jointAngleXZY=None, jointAngleErgo=None,
                 centerOfMass=None, mapping=None, sensors=None, segments=None, joints=None,
                 root=None, mvn=None, comment=None, subject=None, version=None, build=None, label=None,
                 frameRate=None, segmentCount=None, recordingDate=None, configuration=None, userScenario=None,
                 securityCode=None, modality=None, time=None, index=None, timecode=None, ms=None):

        self.verbose = True
        if orientation is None:
            self.orientation = []
        if position is None:
            self.position = []
        if velocity is None:
            self.velocity = []
        if acceleration is None:
            self.acceleration = []
        if angularVelocity is None:
            self.angularVelocity = []
        if angularAcceleration is None:
            self.angularAcceleration = []
        if footContacts is None:
            self.footContacts = []
        if sensorFreeAcceleration is None:
            self.sensorFreeAcceleration = []
        if sensorMagneticField is None:
            self.sensorMagneticField = []
        if sensorOrientation is None:
            self.sensorOrientation = []
        if jointAngle is None:
            self.jointAngle = []
        if jointAngleXZY is None:
            self.jointAngleXZY = []
        if jointAngleErgo is None:
            self.jointAngleErgo = []
        if centerOfMass is None:
            self.centerOfMass = []
        if sensors is None:
            self.sensors = []
        if segments is None:
            self.segments = {}
        if joints is None:
            self.joints = []
        if mapping is None:
            self.mapping = {"orientation": 0,
                            "position": 1,
                            "velocity": 2,
                            "acceleration": 3,
                            "angularVelocity": 4,
                            "angularAcceleration": 5,
                            "footContacts": 6,
                            "sensorFreeAcceleration": 7,
                            "sensorMagneticField": 8,
                            "sensorOrientation": 9,
                            "jointAngle": 10,
                            "jointAngleXZY": 11,
                            "jointAngleErgo": 12,
                            "centerOfMass": 13}
        if time is None:
            self.time = []
        else:
            self.time = time
        if index is None:
            self.index = []
        else:
            self.index = index
        if timecode is None:
            self.timecode = []
        else:
            self.timecode = timecode
        if ms is None:
            self.ms = []
        else:
            self.ms = ms
        self.mvn = mvn
        self.comment = comment
        self.subject = subject
        self.version = version
        self.build = build
        self.label = label
        self.frameRate = frameRate
        self.segmentCount = segmentCount
        self.recordingDate = recordingDate
        self.configuration = configuration
        self.userScenario = userScenario
        self.securityCode = securityCode
        self.modality = modality

        if path is None:
            self.path = os.path.join(dirpath, "recordings/mvnx/all_test.mvnx")
            print(logger.OKBLUE +
                  "[INFO] Parsing file: \t{}".format(self.path) + logger.ENDC)
        else:
            self.path = path

        self.root = None

        # if root is None:
        #     self.parse_mvnx(self.path)
        #     self.parse_all()
        # else:
        #     self.root = root

    def parse_mvnx(self, path):
        """
        Take a path to an MVNX file and parse it

        Args:
            path ([string]): [the path to the data file]
        """
        tree = ET.parse(path)
        self.root = tree.getroot()
        if self.root is None:
            self.root = self.parse_mvnx(self.path)
        self.mvn = self.root[0]
        self.version = self.root[0].attrib['version']
        self.build = self.root[0].attrib['build']
        self.comment = self.root[1].text
        self.securityCode = self.root[3].attrib['code']

        if self.verbose:
            print(logger.OKBLUE + "[INFO] MVNX version: \t{}".
                  format(self.version) + logger.ENDC)
            print(logger.OKBLUE + "[INFO] MVNX build: \t{}".
                  format(self.build) + logger.ENDC)

        return self.root

    def parse_subject(self):
        """Parse information of the subject for Xsens MVN
        """
        if self.root is None:
            self.root = self.parse_mvnx(self.path)

        self.subject = self.root[2]
        self.subject_name = self.subject.attrib['label']
        self.frameRate = self.subject.attrib['frameRate']
        self.segmentCount = self.subject.attrib['segmentCount']
        self.configuration = self.subject.attrib['configuration']

        if self.verbose == True:
            print(logger.OKGREEN +
                  "[INFO] label: \t\t\t{}".format(self.subject_name) + logger.ENDC)
            print(logger.OKGREEN +
                  "[INFO] Frame rate: \t\t{}".format(self.frameRate) + logger.ENDC)
            print(logger.OKGREEN +
                  "[INFO] No. of segments: \t{}".format(self.segmentCount) + logger.ENDC)
            print(logger.OKGREEN +
                  "[INFO] Body Configuration: \t{}".format(self.configuration) + logger.ENDC)
            print(logger.OKGREEN + '-'*50 + logger.ENDC)

    def parse_modality(self, modality):
        """[With a given XML Tree, parse out the salient modalities within each frame]

        Args:

            modality ([string]): [the name of the modality]

        """

        holding_list = []
        frames = self.root[2][3]
        for frame in frames[3:]:
            for child in frame[self.mapping[modality]:self.mapping[modality]+1]:
                holding_list.append(child.text.split(' '))
        holding_list = np.array(holding_list)
        return holding_list.astype(float)

    def parse_time(self):
        frames = self.root[2][3][3:]
        for frame in frames:
            self.time.append(frame.attrib['time'])
        return self.time

    def parse_index(self):
        frames = self.root[2][6][3:]
        for frame in frames:
            self.index.append(frame.attrib['index'])
        return self.index

    def parse_timecode(self):
        frames = self.root[2][6][3:]
        for frame in frames:
            self.timecode.append(frame.attrib['tc'])
        return self.timecode

    def parse_ms(self):
        frames = self.root[2][6][3:]
        for frame in frames:
            self.ms.append(frame.attrib['ms'])
        return self.ms

    def parse_modalities(self, *args):
        for arg in args:
            print(self.parse_modality(arg))
            return self.parse_modality(arg)

    def parse_sensors(self):
        """Parse sensor information of Xsens MVN Analyze

        Returns:
            sensors_information (list): list of sensor names for the Xsens MVN Analyze
        """
        self.sensors = self.root[2][2]
        self.sensors_information = []
        for sensor in self.sensors:
            self.sensors_information.append(sensor.attrib['label'])

        if self.verbose:
            if self.sensors_information == []:
                print(logger.FAIL +
                      "[ERR.] Unable to parse sensor information" + logger.ENDC)
            else:
                print(logger.OKGREEN +
                      "[INFO] Successfully parsed sensor information" + logger.ENDC)

        return self.sensors_information

    def parse_segments(self):
        """Parse segment information and frane offset from the mvnx file

        Returns:
            segment_information (list): Returns the list of custom class segment for the current file
        """

        self.segments = self.root[2][1]
        self.segments_information = []

        for segment in self.segments:
            segment_data = segment_holder()
            segment_data.NAME = segment.attrib['label']
            segment_data.ID = segment.attrib['id']

            # Parse individual connecting joints and points
            joints, positions = [], []
            for c_joint in segment[0]:
                joint = c_joint.attrib['label']

                # Only consider connecting joints and not anatomy joints
                if joint[0] == 'p':
                    continue

                position = c_joint[0].text.split()
                for i in range(len(position)):
                    position[i] = float(position[i])

                joints.append(joint)
                positions.append(position)

            segment_data.cJOINTS = joints
            segment_data.posJOINTS = positions

            self.segments_information.append(segment_data)

        if self.verbose:
            if self.segments_information == []:
                print(logger.FAIL +
                      "[ERR.] Unable to parse segment information" + logger.ENDC)
            else:
                print(logger.OKGREEN +
                      "[INFO] Successfully parsed segment information" + logger.ENDC)

        # for seg in self.segments_information:
        #     print_segment(seg)

        return self.segments_information

    def parse_joints(self):
        """Parse joint information and store in custom specific class

        Returns:
            joint_information (list): Returns the list of specific class - joint
        """
        self.joints = self.root[2][3]
        self.joints_information = []

        for joint in self.joints:
            joint_data = joint_holder()
            joint_data.NAME = joint.attrib['label']
            joint_data.CONNECTOR1 = joint[0].text.split('/')
            joint_data.CONNECTOR2 = joint[1].text.split('/')

            self.joints_information.append(joint_data)
        if self.verbose:
            if self.joints_information == []:
                print(logger.FAIL +
                      "[ERR.] Unable to parse joint information" + logger.ENDC)
            else:
                print(logger.OKGREEN +
                      "[INFO] Successfully parsed joint information" + logger.ENDC)

        # for joint in self.joints_information:
        #     print_joint(joint)

        return self.joints_information

    def parse_frame(self):
        """Parse frame information with details related from Xsens User Manual. For now, only information on position, orientation, acceleration, joint_angle and centre of mass are extracted. 

        Returns:
            frame_information (list): Returns the list of frames at each timestep.
        """
        self.frames = self.root[2][6]
        self.frame_information = []
        count = 1

        for frame in self.frames:
            frame_data = frame_holder()
            # print(logger.OKCYAN +
            #   "[INFO] Processing frame {}...".format(count) + logger.ENDC, flush=True, end="\r")

            # Only parse information on Normal poses
            if frame.attrib['type'] == 'normal':
                frame_data.TIME = frame.attrib['time']
                frame_data.TIMESTAMP = frame.attrib['tc']
                frame_data.ms = frame.attrib['ms']
                frame_data.TYPE = frame.attrib['type']

                # Parse position information
                frame_data.STR_POSITION = frame[FRAME_ID.POSITION].text
                positions = frame[FRAME_ID.POSITION].text.split()
                for i in range(0, len(positions) - 1):
                    positions[i] = float(positions[i])

                positions = np.asarray(positions, dtype=np.float32)
                frame_data.PLOT_POSITIONS = positions
                frame_data.POSITION = positions.reshape(
                    (int(positions.size/3), 3))

                # Parse orientation information
                frame_data.STR_ORIENTATION = frame[FRAME_ID.ORIENTATION].text
                orientations = frame[FRAME_ID.ORIENTATION].text.split()
                for i in range(0, len(orientations) - 1):
                    orientations[i] = float(orientations[i])

                orientations = np.asarray(orientations, dtype=np.float32)
                frame_data.ORIENTATION = orientations.reshape(
                    (int(orientations.size/4), 4))

                # Parse Euler's angle
                joint_angles = frame[FRAME_ID.JOINT_ANGLE].text.split()
                for i in range(0, len(joint_angles) - 1):
                    joint_angles[i] = float(joint_angles[i])

                joint_angles = np.asarray(joint_angles, dtype=np.float32)
                frame_data.JOINT_ANGLE = joint_angles.reshape(
                    (int(joint_angles.size/3), 3))

                # Parse CoM information
                frame_data.STR_CENTRE_OF_MASS = frame[FRAME_ID.CENTRE_OF_MASS].text
                CoM = frame[FRAME_ID.CENTRE_OF_MASS].text.split()
                for i in range(0, len(CoM) - 1):
                    CoM[i] = float(CoM[i])

                CoM = np.asarray(CoM, dtype=np.float32)
                frame_data.PLOT_CoM = CoM
                frame_data.CoM = CoM.reshape((int(CoM.size/3), 3))

                self.frame_information.append(frame_data)
                count += 1

        if self.verbose:
            if self.joints_information == []:
                print(logger.FAIL +
                      "[ERR.] Unable to parse frame information" + logger.ENDC)
            else:
                print(logger.OKGREEN +
                      "[INFO] Successfully parsed frame information" + logger.ENDC)

        return self.frame_information

    def parse_all(self):
        subject = self.parse_subject()
        segments = self.parse_segments()
        sensors = self.parse_sensors()
        joints = self.parse_joints()
        frames = self.parse_frame()

        # self.parse_timecode()
        # self.parse_ms()

        return subject, segments, sensors, joints, frames


def main():
    mvnx = MVNX()
    _, _, _, _, frames = mvnx.parse_all()
    print(frames[0].STR_POSITION)


if __name__ == "__main__":
    main()
