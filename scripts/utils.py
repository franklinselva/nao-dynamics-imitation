

class logger:
    """Logger based colouring for print statements
    """
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class segment:
    """Holds the segment information. In this case, holds the segment information of the connecting joints
    """
    NAME = ""
    cJOINTS = []
    posJOINTS = []
    ID = int()


class joint:
    """Holds the joint information for the human body
    """

    NAME = ""
    CONNECTOR1 = ""
    CONNECTOR2 = ""


class SEGMENT_ID:
    """Used as enumeration and segment ID for broadcasting message with SOCKET
    """
    
    PELVIS = (0, "Pelvis")
    L5 = (1, "L5")
    L3 = (2, "L3")
    T12 = (3, "T12")
    T8 = (4, "T8")
    NECK = (5, "Neck")
    HEAD = (6, "Head")
    RIGHT_SHOULDER = (7, "Right Shoulder")
    RIGHT_UPPER_ARM = (8, "Right Upper Arm")
    RIGHT_FOREARM = (9, "Right Forearm")
    RIGHT_HAND = (10, "Right Hand")
    
    LEFT_SHOULDER = (11, "Left Shoulder")
    LEFT_UPPER_HAND = (12, "Left Upper Arm")
    LEFT_FOREARM = (13, "Left Forearm")
    LEFT_HAND = (14, "Left Hand")
    
    RIGHT_UPPER_LEG = (15, "Right Upper Leg")
    RIGHT_LOWER_LEG = (16, "Right Lower Leg")
    RIGHT_FOOT = (17, "Right Foot")
    RIGHT_TOE = (18, "Right Toe")
    
    LEGT_UPPER_LEG = (19, "Left Upper Leg")
    LEFT_LOWER_LEG = (20, "Left Lower Leg")
    LEFT_FOOT = (21, "Left Foot")
    LEFT_TOE = (22, "Left Toe")
    
class FRAME_ID:
    """Used as enumeration for the frame ID. Lists the available information for a given frame at 1 second.
    """
    ORIENTATION = 0
    POSITION = 1
    VELOCITY = 2
    ACCELERATION = 3
    ANGULAR_VELOCITY = 4
    ANGULAR_ACCELERATION = 5
    FOOT_CONTACT = 6
    SENSOR_FREE_ACCELERATION = 7
    SENSOR_MAGNETIC_FIELD = 8
    SENSOR_ORIENTATION = 9
    JOINT_ANGLE = 10
    JOINT_ANGLE_XYZ = 11
    JOINT_ANGLE_ERGO = 12
    JOINT_ANGLE_ERGO_XYZ = 13
    CENTRE_OF_MASS = 14


class frame:
    TIME = int()
    TIMESTAMP = None
    ms = None
    TYPE = ""

    PLOT_POSITIONS = None
    PLOT_CoM = None

    POSITION = None
    ORIENTATION = None
    VELOCITY = None
    ACCELERATION = None
    JOINT_ANGLE = None
    CENTRE_OF_MASS = None

    STR_POSITION = ""
    STR_CENTRE_OF_MASS = ""
    STR_JOINT_ANGLE = ""
    STR_ACCELERATION = ""
    STR_VELOCITY = ""
    STR_ORIENTATION = ""


def print_segment(segment):
    print(logger.OKCYAN + "Segment Name: \t {}".format(segment.NAME) + logger.ENDC)
    print(logger.OKCYAN + "Segment ID: \t {}".format(segment.ID) + logger.ENDC)
    print(logger.OKCYAN +
          "Connecting Joints: \t {}".format(segment.cJOINTS) + logger.ENDC)


def print_joint(joint):
    print(logger.OKCYAN + "Joint name: \t{}".format(joint.NAME) + logger.ENDC)
    print(logger.OKCYAN +
          "Joint Connector 1: \t{}".format(joint.CONNECTOR1) + logger.ENDC)
    print(logger.OKCYAN +
          "Joint Connector 2: \t{}".format(joint.CONNECTOR2) + logger.ENDC)

import struct
def bytes_conversion_po(position, orientation, segment_id):
    data = struct.pack('>f', position[segment_id][0]) + struct.pack('>f', position[segment_id][1]) + struct.pack('>f', position[segment_id][2]) + \
        struct.pack('>f', orientation[segment_id][0]) + struct.pack('>f', orientation[segment_id][1]) + struct.pack('>f', orientation[segment_id][2]) + struct.pack('>f', orientation[segment_id][3])
    return data

def bytes_conversion_com(centre_of_mass):
    data = struct.pack('>f', centre_of_mass[0][0]) + struct.pack('>f', centre_of_mass[0][1]) + struct.pack('>f', centre_of_mass[0][2]) 
    return data
