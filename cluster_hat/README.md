# Move to see pi client/server

## Information about this code

Simple implementation of a qt ui, with a grid of video streams. Each video stream has its own dedicated thread, and interfaces to exactly one pi.

## Setup and configuration
Each of the pis will need to know the server's IP address. It may be desirable to change the range of ports being used if there is any collisions occuring. Currently the range of ports is '5555 - 5564'. The current image dimensions being used is 200x200.

## Suggested approach to interfacing this code with pre-existing "Move to see code"
Blank for now

### Installation and configuration
####Clusterhat softare and configuration
Immediately upon installation, it is strongly suggested that the partitions of the SD card be expanded. The default partition size provided by the clusterhat images is only 1.7Gb, which runs out very quickly. I found the following page to be extremely useful for this:
> https://raspberrypi.stackexchange.com/questions/499/how-can-i-resize-my-root-partition
####Interfacing with the PI zeros.
Once you get VNC/SSH access to the controller pi, interfacing the pi zeros is a straightforward matter. Note that the cluster will usually not have the pi zeros enabled upon startup, so this will need to be done manually. There are two primary methods of enabling the clusters. The first is through the shell:
>pi@controller:~$clusterhat on p1 (1-4)

The second is through a simple gui interface, found through the start menu-> accessories -> clusterhat.

Once enabled, you can interface with the zeros directly from the controller pi by using:
>pi@controller:~$ minicom p1 (1-4)

This will grant you access to the cluster pi. Note that the pi's will have the ssh interface disabled by default. You will need to run:
>pi@p1:~$ sudo touch /boot/ssh
>pi@p1:~$ sudo reboot

Now you should be able to access all of the pis through ssh.

#### Installing ROS
Installing ros is a very straightforward matter once you have the clusterhat software installed, following this guide:
> https://gist.github.com/Tiryoh/ce64ad0d751a9c298b87dc059d6cca37

I was very hesistant to attempt this approach, as it seemed like it would deliver out-dated software. This isn't an issue, as once the software is installed, it can update without any issues. I tried at least three other approaches, but this was the only approach that worked reliably.

####Running the provided client/server software.
You will need to collect some software in order to run these.
>pi@p1:~$sudo apt-get install libcblas-dev libhdf5-dev libhdf5-serial-dev libatlas-base-dev libjasper-dev libqtgui4 libqt4-test python-opencv

>opencv-pythonpi@p1:~$sudo pip install zmq

## Clusterhat Tips and Troubleshooting
### Errors:
#### "Camera.grab() is returning false":
This seems to be either caused by a module not being loaded upon boot, OR, you are missing some dependencies. I am unsure about what EXACTLY causes this, but the fix is simple:
> pi@p1:~$sudo modprobe bcm2835-v4l2

OR

> pi@p1:~$sudo apt-get install libcblas-dev libhdf5-dev libhdf5-serial-dev libatlas-base-dev libjasper-dev libqtgui4 libqt4-test

#### Video stream seems to be lagging
The fix for this is simply to restart the server. Given more time, I would love to attempt an implementation of an error check for this. My approach to this would be to attatch timestamps to the frames being sent to from the pis, and have the servers cross-check these time-stamps. If something seems wrong, just have the server reboot the thread belonging to the lagging pi, which should (In theory) have no effect on the surrounding pis.

