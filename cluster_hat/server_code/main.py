import cv2
import sys
from libs.server_interface import server_interface
from PyQt5 import QtCore
from PyQt5 import QtWidgets
from PyQt5 import QtGui


class ShowVideo(QtCore.QObject):

    VideoSignal = QtCore.pyqtSignal(QtGui.QImage)

    def __init__(self, parent=None):
        super(ShowVideo, self).__init__(parent)
        # Initialize the interface to the server
        self.servInterface = server_interface()

    @QtCore.pyqtSlot()
    def startVideo(self):

        run_video = True
        while run_video:
            # Instruct the server to retrieve the latest image
            self.servInterface.recieve_image()
            # Retrieve the latest image from the server interface
            image = self.servInterface.source

            color_swapped_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            height, width, _ = color_swapped_image.shape
            qt_image = QtGui.QImage(color_swapped_image.data,
                                    width,
                                    height,
                                    color_swapped_image.strides[0],
                                    QtGui.QImage.Format_RGB888)

            self.VideoSignal.emit(qt_image)


class ImageViewer(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(ImageViewer, self).__init__(parent)
        self.image = QtGui.QImage()
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.drawImage(0, 0, self.image)
        self.image = QtGui.QImage()

    def initUI(self):
        self.setWindowTitle('QUT Camera thing')

    @QtCore.pyqtSlot(QtGui.QImage)
    def setImage(self, image):
        if image.isNull():
            print("Viewer Dropped frame!")

        self.image = image
        # Check the dimensions of the image, make sure the frame matches this
        if image.size() != self.size():
            self.setFixedSize(image.size())
        self.update()


class VidStreamObject():
    def __init__(self, socket_address):
        self.thread = QtCore.QThread()
        self.thread.start()
        self.vid_obj = ShowVideo()
        self.vid_obj.servInterface.socket_address = socket_address
        self.vid_obj.servInterface.bind_socket()
        self.vid_obj.moveToThread(self.thread)
        self.image_obj = ImageViewer()
        self.vid_obj.VideoSignal.connect(self.image_obj.setImage)


if __name__ == '__main__':

    app = QtWidgets.QApplication(sys.argv)

    # Begin init for the screen objects
    screen_p1 = VidStreamObject('tcp://*:5556')
    screen_p2 = VidStreamObject('tcp://*:5557')
    screen_p3 = VidStreamObject('tcp://*:5558')
    screen_p4 = VidStreamObject('tcp://*:5559')
    screen_p5 = VidStreamObject('tcp://*:5560')
    screen_p6 = VidStreamObject('tcp://*:5561')
    screen_p7 = VidStreamObject('tcp://*:5562')
    screen_p8 = VidStreamObject('tcp://*:5563')

    # Button to start the videocapture:
    push_button = QtWidgets.QPushButton('Start')
    # Link the buttons to the start video functions
    push_button.clicked.connect(screen_p1.vid_obj.startVideo)
    push_button.clicked.connect(screen_p2.vid_obj.startVideo)
    push_button.clicked.connect(screen_p3.vid_obj.startVideo)
    push_button.clicked.connect(screen_p4.vid_obj.startVideo)
    push_button.clicked.connect(screen_p5.vid_obj.startVideo)
    push_button.clicked.connect(screen_p6.vid_obj.startVideo)
    push_button.clicked.connect(screen_p7.vid_obj.startVideo)
    push_button.clicked.connect(screen_p8.vid_obj.startVideo)

    # Set the layout to be a grid, this makes assigning the screens to a
    # position easy
    Layout = QtWidgets.QGridLayout()

    # Create the frames for the windows
    Layout.addWidget(screen_p1.image_obj, 0, 0)
    Layout.addWidget(screen_p2.image_obj, 0, 1)
    Layout.addWidget(screen_p3.image_obj, 0, 2)
    Layout.addWidget(screen_p4.image_obj, 1, 0)
    Layout.addWidget(screen_p5.image_obj, 1, 1)
    Layout.addWidget(screen_p6.image_obj, 1, 2)
    Layout.addWidget(screen_p7.image_obj, 2, 0)
    Layout.addWidget(screen_p8.image_obj, 2, 1)

    # Add the button to the ui
    Layout.addWidget(push_button, 3, 1)

    layout_widget = QtWidgets.QWidget()
    layout_widget.setLayout(Layout)

    main_window = QtWidgets.QMainWindow()
    main_window.setCentralWidget(layout_widget)
    main_window.show()
    sys.exit(app.exec_())
