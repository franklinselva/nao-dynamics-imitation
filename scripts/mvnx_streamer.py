from os import remove
from mvnx_parser import MVNX
from utils import logger, bytes_conversion_com, bytes_conversion_po
from utils import SEGMENT_ID
from time import sleep
from sys import getsizeof
import socket
import struct


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
        self.sample_counter = 0
        self.payload_size = None
        self.size_JOINT_DATA = 32
        self.size_JOINT_COM = 12

        self.START = True
        self.ID = 0

        try:
            self.sock.bind((SERVER_IP, SERVER_PORT))
        except Exception as e:
            print(logger.FAIL +
                  "[ERR.] Unable to bind socket: {}".format(e) + logger.ENDC)
    
    def publish_NPOSE(self):
        msg = ""
        msg_prefix = "MXTP20"
        msg = msg_prefix + self.frames[self.ID].STR_POSITION
        try:
            self.sock.sendto(msg.encode('utf-8'), (SERVER_IP, SERVER_PORT))

        except Exception as e:
            print(
                logger.FAIL + "[ERR.] Unable to stream data: {}".format(e) + logger.ENDC)

    def send_message(self):
        self.sample_counter +=1
        
        # Standard data
        self.datagram_counter = struct.pack("!i", 0)
        self.character_id = struct.pack("!i", 0)
        self.no_body_segments = struct.pack("!i", 23)
        self.prop_segments = struct.pack("!i", 0)
        self.finger_tracking = struct.pack("!i", 0)
        
        self.time_code = self.frames[self.ID].TIME.encode('utf-8')
        
        self.no_items = struct.pack("!i", 0)
        self.no_props = struct.pack("!i", 0)
        self.no_finger_tracking = struct.pack("!i", 0)
        
        self.reserved_data = None

        if self.START:
            print(logger.OKGREEN + "[INFO] Started broadcasting messages" + logger.ENDC)
            self.publish_NPOSE()
            self.START = False

        if PUB_JOINTS:
            msg = ""
            msg_prefix = "MXTP02"
            for i in range (0, 23):
                data = bytes_conversion_po(self.frames[self.ID].POSITION, self.frames[self.ID].ORIENTATION, i)

                
                self.payload_size = getsizeof(data)
                self.reserved_data = self.size_JOINT_DATA - self.payload_size - getsizeof(msg_prefix)
                
                msg = msg_prefix.encode('utf-8') + struct.pack("!i", self.sample_counter) + self.datagram_counter + \
                    self.no_items + self.time_code + self.character_id + self.no_body_segments + self.no_props + \
                    self.no_finger_tracking + struct.pack("!i", self.reserved_data) + struct.pack("!i", self.payload_size) + data
                    
                print (msg)
                
                try:
                    self.sock.sendto(msg, (SERVER_IP, SERVER_PORT))

                except Exception as e:
                    print(
                        logger.FAIL + "[ERR.] Unable to stream data: {}".format(e) + logger.ENDC)

        if PUB_CoM:
            msg = ""
            msg_prefix = "MXTP24"
            data = bytes_conversion_com(self.frames[self.ID].CoM)
            

            self.payload_size = getsizeof(data)
            self.reserved_data = self.size_JOINT_DATA - self.payload_size - getsizeof(msg_prefix)
            
            msg = msg_prefix.encode('utf-8') + struct.pack("!i", self.sample_counter) + self.datagram_counter + \
                self.no_items + self.time_code + self.character_id + self.no_body_segments + self.no_props + \
                self.no_finger_tracking + struct.pack("!i", self.reserved_data) + struct.pack("!i", self.payload_size) + data

            print (msg)
            try:
                self.sock.sendto(msg, (SERVER_IP, SERVER_PORT))
            except Exception as e:
                print(logger.FAIL + "[ERR.] Unable to stream data" + logger.ENDC)

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
