
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


def print_segment(segment):
    print(logger.OKCYAN + "Segment Name: \t {}".format(segment.NAME) + logger.ENDC)
    print(logger.OKCYAN + "Segment ID: \t {}".format(segment.ID) + logger.ENDC)
    print(logger.OKCYAN +
          "Connecting Joints: \t {}".format(segment.cJOINTS) + logger.ENDC)
