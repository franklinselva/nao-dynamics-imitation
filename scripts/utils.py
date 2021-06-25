

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
