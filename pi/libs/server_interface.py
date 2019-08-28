import cv2
import zmq
import base64
import numpy as np

import threading

import rospy
from sensor_msgs.msg import Image, CompressedImage

class server_interface():

    def __init__(self, topic, lock):
        print("Creating subscriber to topic: "+topic)
        self.lock = lock
        self.topic = topic
        self.subscriber = rospy.Subscriber(topic, CompressedImage, self.recieve_image,  queue_size = 1)
        self.connected = False
        # self.context = zmq.Context()
        # self.footage_socket = self.context.socket(zmq.SUB)
        # self.socket_address = ''

    # def bind_socket(self):
        # self.footage_socket.bind(self.socket_address)
        # self.footage_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))

    def recieve_image(self, ros_data):
        # print("Got image callback")
        # frame = self.footage_socket.recv_string()
        # img = base64.b64decode(frame)
        # npimg = np.fromstring(img, dtype=np.uint8)
        # self.source = cv2.imdecode(npimg, 1)
        self.connected = True

        # print 'received image of type: "%s"' % ros_data.format
        np_arr = np.fromstring(ros_data.data, np.uint8)
        self.lock.acquire()
        self.source = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.lock.release()
        #
        # cv2.imshow("Test", self.source)
        # cv2.waitKey(30)
        # self.source = cv2.imdecode(np_arr, 1)

    # def get_frame(self):
    #
    #     return self.source
