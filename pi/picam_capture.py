import RPi.GPIO as gp
from picamera import PiCamera
import os
import numpy as np
import time

print "capturing picam image A"
gp.setwarnings(False)
gp.setmode(gp.BOARD)

gp.setup(7, gp.OUT)
gp.setup(11, gp.OUT)
gp.setup(12, gp.OUT)

gp.setup(15, gp.OUT)
gp.setup(16, gp.OUT)
gp.setup(21, gp.OUT)
gp.setup(22, gp.OUT)

gp.output(11, True)
gp.output(12, True)
gp.output(15, True)
gp.output(16, True)
gp.output(21, True)
gp.output(22, True)


def setIO(cameraIDX):
    gp.output(11, True)
    gp.output(12, True)
    gp.output(15, True)
    gp.output(16, True)
    gp.output(21, True)
    gp.output(22, True)

    if (cameraIDX < 4):
        if (cameraIDX == 0):
            print "Board 1 Camera B"
            gp.output(7, True)
            gp.output(11, False)
            gp.output(12, True)
        if (cameraIDX == 1):
            print "Board 1 Camera C"
            gp.output(7, False)
            gp.output(11, True)
            gp.output(12, False)
        if (cameraIDX == 2):
            print "Board 1 Camera D"
            gp.output(7, True)
            gp.output(11, True)
            gp.output(12, False)
        if (cameraIDX == 3):
            print "Board 1 Camera A"
            gp.output(7, False)
            gp.output(11, False)
            gp.output(12, True)
    else:
        if (cameraIDX < 8):
            if (cameraIDX == 4):
                print "Board 2 Camera B"
                gp.output(7, True)
                gp.output(15, False)
                gp.output(16, True)
            if (cameraIDX == 5):
                print "Board 2 Camera C"
                gp.output(7, False)
                gp.output(15, True)
                gp.output(16, False)
            if (cameraIDX == 6):
                print "Board 2 Camera D"
                gp.output(7, True)
                gp.output(15, True)
                gp.output(16, False)
            if (cameraIDX == 7):
                print "Board 2 Camera A"
                gp.output(7, False)
                gp.output(15, False)
                gp.output(16, True)
        else:
            if (cameraIDX < 12):
                if (cameraIDX == 8):
                    print "Board 3 Camera B"
                    gp.output(7, True)
                    gp.output(21, False)
                    gp.output(22, True)
                if (cameraIDX == 9):
                    print "Board 3 Camera C"
                    gp.output(7, False)
                    gp.output(21, True)
                    gp.output(22, False)
                if (cameraIDX == 10):
                    print "Board 3 Camera D"
                    gp.output(7, True)
                    gp.output(21, True)
                    gp.output(22, False)
                if (cameraIDX == 11):
                    print "Board 3 Camera A"
                    gp.output(7, False)
                    gp.output(21, False)
                    gp.output(22, True)

def main():
    for cameraNo in range(8):
        setIO(cameraNo)
        capture(cameraNo)

def capture(id):
    cmd = "raspistill -t 1 -n -o capture_%d.jpg -h 320 -w 240" % id
    os.system(cmd)

    #time.sleep(2)
    #output = np.empty((240, 320, 3), dtype=np.uint8)
    #camera.capture(output,'rgb')

    #outstr = "foo_"+str(id)+".jpg"
    #camera.capture(outstr)

if __name__ == "__main__":

    #camera = PiCamera()
    #camera.resolution = (320, 240)
    #camera.framerate = 24
    main()

    gp.output(7, False)
    gp.output(11, False)
    gp.output(12, True)