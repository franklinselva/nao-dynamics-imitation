from os import remove
from mvnx_parser import MVNX
from utils import logger

import socket


SERVER_PORT = 9763
SERVER_IP = "127.0.0.1"

PUB_JOINTS = True
PUB_CoM = True
PUB_QUAT = True


class mvnx_streamer():
    """UDP Client for sending xsens message from recorded MVNX file
    """

    def __init__(self):
        self.mvnx = MVNX()
        _, self.segments, self.sensors, self.joints, self.frames = self.mvnx.parse_all()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)

        try:
            self.sock.accept((SERVER_IP, SERVER_PORT))
        except Exception as e:
            print(logger.FAIL +
                  "[ERR.] Unable to bind socket: {}".format(e) + logger.ENDC)

    def encode_message(self):
        msg = ""
        msg_prefix = "MXTP"

        if PUB_JOINTS:
            msg_prefix = msg_prefix + "20"
            yield msg

        if PUB_CoM:
            msg_prefix = msg_prefix + "24"
            yield msg

        if PUB_QUAT:
            msg_prefix = msg_prefix + "02"
            yield msg

    def stream_udp(self):
        """Stream Xsens data parsed from recorded MVNX files using UDP Protocol 
        """
        try:
            while True:
                msg = self.encode_message()
                try:
                    self.sock.sendto(msg.encode('utf-8'),
                                     (SERVER_IP, SERVER_PORT))
                except Exception as e:
                    print(logger.FAIL + "[ERR.] Unable to stream data")
        except KeyboardInterrupt:
            print(logger.OKGREEN + "[INFO] Closing connection" + logger.ENDC)
            exit(2)


if __name__ == "__main__":
    mvnx = mvnx_streamer()
    mvnx.stream_udp()
