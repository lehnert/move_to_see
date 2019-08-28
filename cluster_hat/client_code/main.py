# Example of code being used on the PI's
import base64
import cv2
import zmq
import sys

context = zmq.Context()
footage_socket = context.socket(zmq.PUB)
footage_socket.connect('tcp://192.168.0.21:5555')
'''
This socket needs to be changed depending on the pi. two pis must not share the same socket value, else
there will be address conflicts
'''

camera = cv2.VideoCapture(0)  # init the camera
if camera.open():
    print("Succesfully accessed camera")
else:
    print("Couldn't open the camera, check physical connection, or refer to readme")
camera.open(-1)

while True:
    try:
        grabbed, frame = camera.read()  # grab the current frame
        if grabbed:
            frame = cv2.resize(frame, (200, 200))  # resize the frame
            encoded, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer)
            footage_socket.send(jpg_as_text)

    except KeyboardInterrupt:
        camera.release()
        cv2.destroyAllWindows()
        break
