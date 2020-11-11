
from PyQt5.QtWidgets import (QWidget, QSlider, QHBoxLayout, QVBoxLayout,
                             QLabel, QApplication)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
import PyQt5
import sys
import pyqtgraph as pg
import sys

class CustomGraphicsWindow(pg.GraphicsWindow):
    # Function closeEvent
    #	This overrides the closeEvent function in the base class. This function is invoked automatically by QTGui when the window closes.
    #	@param ev This is the event object (i.e. window close).
    def closeEvent(self, ev):
        print ("Exiting gracefully...")
        sys.exit()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Q:
            print ("Exiting gracefully...")
            sys.exit()
        if event.key() == Qt.Key_Space:
            self.v_sld.setValue(0)
            self.w_sld.setValue(0)
        if event.key() == Qt.Key_I:
            self.v_sld.setValue(self.v+10)
        if event.key() == Qt.Key_K:
            self.v_sld.setValue(self.v-10)
        if event.key() == Qt.Key_J:
            self.w_sld.setValue(self.w-10)
        if event.key() == Qt.Key_L:
            self.w_sld.setValue(self.w+10)
        if event.key() == Qt.Key_U:
            self.v_sld.setValue(0)
        if event.key() == Qt.Key_O:
            self.w_sld.setValue(0)


class BaseRobotControlWindow(CustomGraphicsWindow):
    def __init__(self):
        super().__init__()

        self.v = 0
        self.w = 0

        self.initUI()

    def initUI(self):
        vbox = QVBoxLayout()

        self.v_sld = QSlider(Qt.Vertical, self)
        self.v_sld.setRange(-100, 100)
        self.v_sld.setFocusPolicy(Qt.NoFocus)
        self.v_sld.setPageStep(10)
        self.v_sld.valueChanged.connect(self.update_linear_vel)

        self.v_label = QLabel('0', self)
        self.v_label.setStyleSheet("QLabel { color : yellow; }")
        self.v_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.v_label.setMinimumWidth(80)

        self.w_sld = QSlider(Qt.Horizontal, self)
        self.w_sld.setRange(-100, 100)
        self.w_sld.setFocusPolicy(Qt.NoFocus)
        self.w_sld.setPageStep(10)
        self.w_sld.valueChanged.connect(self.update_angular_vel)


        self.w_label = QLabel('0', self)
        self.w_label.setStyleSheet("QLabel { color : yellow; }")
        self.w_label.setAlignment(Qt.AlignCenter | Qt.AlignVCenter)
        self.w_label.setMinimumWidth(80)

        msg = """
        Use the sliders or the keyboard map as described below to control your real robot.
        ---------------------------
        Moving around:
             i    
        j    k    l


        anything else : stop

        u : Set linear speed to 0
        o : Set angular speed to 0
        space : Set linear and angular speeds to 0
        q : Quit

        """
        self.info_label = QLabel(msg, self)
        self.info_label.setStyleSheet("QLabel { background-color : gray; color : yellow; }")
        vbox.addWidget(self.info_label)


        vbox.addWidget(self.w_sld)
        vbox.addSpacing(15)
        vbox.addWidget(self.w_label)
        self.setLayout(vbox)

        vbox.addWidget(self.v_sld, alignment=Qt.AlignHCenter)
        vbox.addSpacing(15)
        vbox.addWidget(self.v_label)
        self.setLayout(vbox)


        self.setWindowTitle('QSlider')
        self.show()

    def update_linear_vel(self, value):
        self.v = value
        self.v_label.setText(str(self.v))
        self.publish_linear_vel(value)
    
    def update_angular_vel(self, value):
        self.w = value
        self.w_label.setText(str(self.w))
        self.publish_angular_vel(value)


class RobotControlWindow(BaseRobotControlWindow):
    """A GUI to set the linear and angular speeds of the robot.
    """
    def __init__(self):
        super().__init__()
    
    def publish_linear_vel(self, value):
        """When the linear speed slider is changed, this function is called.
        Add the bluetooth command that controls the linear speed here.

        Args:
            value ([integer]): An integer ranging in the range [-100, 100] and has the current value 
                               of the linear speed slider 
        """
        print(" | Publishing...")
        print("v = {:.3f} | w = {:.3f}".format(self.v, self.w))

    
    def publish_angular_vel(self, value):
        """When the angular speed slider is changed, this function is called.
        Add the bluetooth command that controls the angular speed here.

        Args:
            value ([integer]): An integer ranging in the range [-100, 100] and has the current value 
                               of the angular speed slider
        """
        print(" | Publishing...")
        print("v = {:.3f} | w = {:.3f}".format(self.v, self.w))


def main():
    app = QApplication(sys.argv)
    robot_control_app = RobotControlWindow()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()