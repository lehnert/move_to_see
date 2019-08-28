import cv2
import zmq
import base64
import numpy as np


class server_interface():

    def __init__(self):
        self.context = zmq.Context()
        self.footage_socket = self.context.socket(zmq.SUB)
        self.socket_address = ''

    def bind_socket(self):
        self.footage_socket.bind(self.socket_address)
        self.footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))

    def recieve_image(self):
        frame = self.footage_socket.recv_string()
        img = base64.b64decode(frame)
        npimg = np.fromstring(img, dtype=np.uint8)
        self.source = cv2.imdecode(npimg, 1)
