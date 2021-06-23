import numpy as np
import xml.etree.ElementTree as ET
import os

from utils import logger, print_segment
from utils import segment as segment_holder

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
            print('Path is been provided. Taking default path')
            self.path = os.path.join(dirpath, "recordings/mvnx/all_test.mvnx")
        else:
            self.path = path

        if root is None:
            self.parse_mvnx(self.path)
            self.parse_all()
        else:
            self.root = root

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
        if self.root is None:
            self.root = self.parse_mvnx(self.path)

        self.subject = self.root[2]
        self.subject_name = self.subject.attrib['label']
        self.frameRate = self.subject.attrib['frameRate']
        self.segmentCount = self.subject.attrib['segmentCount']
        self.configuration = self.subject.attrib['configuration']

        if self.verbose == True:
            print(logger.OKGREEN +
                  "[INFO] label: \t{}".format(self.subject_name) + logger.ENDC)
            print(logger.OKGREEN +
                  "[INFO] Frame rate: \t{}".format(self.frameRate) + logger.ENDC)
            print(logger.OKGREEN +
                  "[INFO] No. of segments: \t{}".format(self.segmentCount) + logger.ENDC)
            print(logger.OKGREEN +
                  "[INFO] Body Configuration: \t{}".format(self.configuration) + logger.ENDC)

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
        self.sensors = self.root[2][2]
        for sensor in self.sensors:
            self.sensors_data.append(sensor.attrib['label'])
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

        return self.segments, self.segments_information

    def parse_joints(self):
        self.joints = self.root[2][3]
        for joint in self.joints:
            self.joints.append(joint.attrib['label'])
        return self.joints

    def parse_all(self):
        self.parse_subject()
        self.parse_segments()
        self.parse_sensors()
        self.parse_joints()
        for key in self.mapping.keys():
            setattr(self, key, self.parse_modality(key))

        self.parse_time()

        # self.parse_timecode()
        # self.parse_ms()


def main():
    mvnx = MVNX()


if __name__ == "__main__":
    main()
