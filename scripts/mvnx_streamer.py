from os import remove
from mvnx_parser import MVNX
from utils import logger
from time import sleep

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

        self.sock = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)  # (, , 0)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.START = True
        self.ID = 0

        try:
            self.sock.bind((SERVER_IP, SERVER_PORT))
        except Exception as e:
            print(logger.FAIL +
                  "[ERR.] Unable to bind socket: {}".format(e) + logger.ENDC)

    def send_message(self):
        if self.START:
            print(logger.OKGREEN + "[INFO] Started broadcasting messages")
            self.START = False

        if PUB_JOINTS:
            msg = ""
            msg_prefix = "MXTP20"
            msg = msg_prefix + self.frames[self.ID].STR_POSITION
            try:
                self.sock.sendto(msg.encode('utf-8'), (SERVER_IP, SERVER_PORT))

            except Exception as e:
                print(
                    logger.FAIL + "[ERR.] Unable to stream data: {}".format(e) + logger.ENDC)

        if PUB_CoM:
            msg = ""
            msg_prefix = "MXTP24"
            msg = msg_prefix + self.frames[self.ID].STR_CENTRE_OF_MASS
            try:
                self.sock.sendto(msg.encode('utf-8'),
                                 (SERVER_IP, SERVER_PORT))
            except Exception as e:
                print(logger.FAIL + "[ERR.] Unable to stream data")

        if PUB_QUAT:
            msg = ""
            msg_prefix = "MXTP02"
            msg = msg_prefix + self.frames[self.ID].STR_ORIENTATION
            try:
                self.sock.sendto(msg.encode('utf-8'),
                                 (SERVER_IP, SERVER_PORT))
            except Exception as e:
                print(logger.FAIL + "[ERR.] Unable to stream data")

        sleep(1)  # Sending messages per second

        self.ID += 1

    def stream_udp(self):
        """Stream Xsens data parsed from recorded MVNX files using UDP Protocol 
        """
        try:
            while True:
                self.send_message()
        except KeyboardInterrupt:
            print(logger.OKGREEN + "[INFO] Closing connection" + logger.ENDC)
            exit(2)


if __name__ == "__main__":
    mvnx = mvnx_streamer()
    mvnx.stream_udp()
