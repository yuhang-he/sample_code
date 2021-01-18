from __future__ import print_function
import logging
import sys
import math
import time
import json

from bosdyn.client import create_standard_sdk, RpcError
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.client.util
import bosdyn.geometry as geometry
from bosdyn.util import secs_to_hms

from PySide2.QtWidgets import QApplication, QTableWidgetItem, QDialog, QVBoxLayout, QPushButton, QTextBrowser, \
    QFileDialog
from PySide2.QtCore import Signal, Slot, QEvent, Qt
from src.qt_gui import MainWindow
from src.spot_core import SpotCore
from src.camera_control import CameraControl
from src.waypoint import WayPointControl
from src.mobility_params import MobilityParams
from src.ricoh_theta import RicohThetaControl
from src.hololens2 import HoloWebSocket
from src.faro_scan import ScannerControl
from src.pan_tilt import ArduinoWebSocket
from google.protobuf.json_format import MessageToJson

LOGGER = logging.getLogger()


class SpotInterface(object):
    """Main Qt GUI class that is responsible for communication between GUI interface and Spot API

    Args:
        robot: robot instances created from sdk.create_robot
    """

    def __init__(self):
        """Create QApplication and QMainWindow"""
        self.app = QApplication(sys.argv)
        try:
            self.main_window = MainWindow("src/mainwindow.ui")
        except:
            self.main_window = MainWindow("SpotDashBoard/src/mainwindow.ui")
        self.window = self.main_window.window
        self.window.open_token.clicked.connect(self.open_token)
        self.window.save_login.clicked.connect(self.save_connection)
        self.window.spot_connect.clicked.connect(self.connect_spot)

        self.scanner = ScannerControl(self.window)
        self.window.rtv_enable.clicked.connect(self.rtv_enable)
        self.window.holo_enable.clicked.connect(self.holo_enable)
        self.window.pan_tilt_enable.clicked.connect(self.pan_tilt_enable)

        try:
            self.load_connection()
        except:
            self.window.qtlog.append("Cannot load saved connection setting")

    def open_token(self):
        self.window.spot_token_path.setText(self.open_file_name("Open Spot token"))

    def save_connection(self):
        with open("connect.JSON", "w") as file:
            connection = {"hostname": self.window.spot_hostname.text(), "user": self.window.spot_username.text(),
                          "password": self.window.spot_password.text(), "token": self.window.spot_token_path.text()}
            connection = json.dumps(connection, indent=4)
            file.write(connection)

    def load_connection(self):
        with open("connect.JSON", "r") as file:
            connection = json.loads(file.read())
            self.window.spot_hostname.setText(connection["hostname"])
            self.window.spot_username.setText(connection["user"])
            self.window.spot_password.setText(connection["password"])
            self.window.spot_token_path.setText(connection["token"])

    # Create robot object.
    def connect_spot(self):
        sdk = create_standard_sdk('SpotGUI')
        sdk.load_app_token(self.window.spot_token_path.text())
        robot = sdk.create_robot(self.window.spot_hostname.text())

        try:
            robot.authenticate(self.window.spot_username.text(), self.window.spot_password.text())
            robot.start_time_sync()
            self.initialize(robot)
        except RpcError as err:
            self.window.qtlog.append("Failed to communicate with robot: {}".format(err))
            return False

    def initialize(self, robot):
        self.rtv_control = None

        # Create SpotCore object that controls Spot
        self.spot_core = SpotCore(robot)

        # Connect Signals from SpotCore object with functions in GUI
        self.spot_core.log.connect(self._log_update)  # Update log after receiving signal
        self.spot_core.stat_update.connect(self._status_update)  # Update status after receiving signal

        # Create CameraControl object to handle camera and video streaming
        self.camera = CameraControl(self.spot_core)

        # Create WayPointControl object to program Spot to follow a trajectory
        self.waypointcontrol = WayPointControl(self.window.waypoint_table, self.spot_core)

        # Create MobilityParams object to update mobility params to Spot locomotion
        self.mobility_control = MobilityParams(self.window, self.spot_core)

        self.scanner = ScannerControl(self.window)

        # Connect the buttons from GUI with functions from SpotCore, Camera, WayPointControl
        self.window.btn_observe.clicked.connect(self.spot_core.observe)
        self.window.btn_control.clicked.connect(self.spot_core.control)
        self.window.btn_e_stop.clicked.connect(self.spot_core.toggle_estop)
        self.window.btn_power.clicked.connect(self.spot_core.toggle_power)
        self.window.btn_self_right.clicked.connect(self.spot_core.self_right)
        self.window.btn_sit.clicked.connect(self.spot_core.sit)
        self.window.btn_stand.clicked.connect(self.spot_core.stand)
        self.window.btn_time_sync.clicked.connect(self.spot_core.toggle_time_sync)
        self.window.btn_left.clicked.connect(self.spot_core.strafe_left)
        self.window.btn_right.clicked.connect(self.spot_core.strafe_right)
        self.window.btn_up.clicked.connect(self.spot_core.move_forward)
        self.window.btn_down.clicked.connect(self.spot_core.move_backward)
        self.window.btn_turn_left.clicked.connect(self.spot_core.turn_left)
        self.window.btn_turn_right.clicked.connect(self.spot_core.turn_right)
        self.window.btn_quit.clicked.connect(self.spot_core.safe_power_off)
        self.window.btn_shutdown.clicked.connect(self.spot_core.shutdown)
        self.window.keyboard_control.clicked.connect(self._toggle_key)
        self.window.btn_camera_cap.clicked.connect(self.camera.take_image)
        self.window.btn_camera_str.clicked.connect(self.camera.toggle_video_mode)

        self.window.btn_face_track_toggle.clicked.connect(self.camera.toggle_face_track)
        self.window.frontR_camera_toggle.toggled.connect(self.camera.frontR_camera_toggle)
        self.window.frontL_camera_toggle.toggled.connect(self.camera.frontL_camera_toggle)
        self.window.left_camera_toggle.toggled.connect(self.camera.left_camera_toggle)
        self.window.right_camera_toggle.toggled.connect(self.camera.right_camera_toggle)
        self.window.back_camera_toggle.toggled.connect(self.camera.back_camera_toggle)

        self.window.frontR_camera_toggle_depth.toggled.connect(self.camera.frontR_camera_toggle_depth)
        self.window.frontL_camera_toggle_depth.toggled.connect(self.camera.frontL_camera_toggle_depth)
        self.window.left_camera_toggle_depth.toggled.connect(self.camera.left_camera_toggle_depth)
        self.window.right_camera_toggle_depth.toggled.connect(self.camera.right_camera_toggle_depth)
        self.window.back_camera_toggle_depth.toggled.connect(self.camera.back_camera_toggle_depth)

        self.window.joint_record.clicked.connect(self._toggle_joint_record)
        self.window.state_record.clicked.connect(self._toggle_state_record)
        self.window.waypoint_add.clicked.connect(self.waypointcontrol.add_row)
        self.window.waypoint_save.clicked.connect(self.waypointcontrol.save)
        self.window.waypoint_execute.clicked.connect(self.waypointcontrol.go_to_waypoint)
        self.window.waypoint_stop.clicked.connect(self.waypointcontrol.stop_execution)

        self._record_joint = False
        self._record_state = False

        # Connect keyboard keys with commands from SpotCore
        self._command_dictionary = {
            Qt.Key_Tab: self.spot_core.safe_power_off,
            Qt.Key_T: self.spot_core.toggle_time_sync,
            Qt.Key_Space: self.spot_core.toggle_estop,
            Qt.Key_R: self.spot_core.self_right,
            Qt.Key_P: self.spot_core.toggle_power,
            Qt.Key_V: self.spot_core.sit,
            Qt.Key_F: self.spot_core.stand,
            Qt.Key_W: self.spot_core.move_forward,
            Qt.Key_S: self.spot_core.move_backward,
            Qt.Key_A: self.spot_core.strafe_left,
            Qt.Key_D: self.spot_core.strafe_right,
            Qt.Key_Q: self.spot_core.turn_left,
            Qt.Key_E: self.spot_core.turn_right,
        }

        # Display main GUI and execute QApplication
        self.window.show()

    def open_file_name(self, desc):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self.window, "{}".format(desc), "",
                                                   "All Files (*);;Python Files (*.py)", options=options)
        if file_name:
            return file_name

    def rtv_enable(self):
        self.rtv_control = RicohThetaControl(self.main_window.window)

    def holo_enable(self):
        self.holo_ws = HoloWebSocket(self.spot_core, self.window)

    def pan_tilt_enable(self):
        self.pan_tilt = ArduinoWebSocket(self.window)

    def _power_state(self):
        """Checking Power State

        Returns:
            robot's motor_power_state
        """
        state = self.spot_core.robot_state
        if not state:
            return None
        return state.power_state.motor_power_state

    def _lease_str(self):
        """Checking and displaying lease status"""
        try:
            alive = 'RUNNING' if self.spot_core._lease_keepalive.is_alive() else 'STOPPED'
            # lease = '{}:{}'.format(self.spot_core._lease.lease_proto.resource, self.spot_core._lease.lease_proto.sequence)
            self.window.lease_stat.setText(alive)  # Display status with QLabel object on GUI
        except AttributeError:
            self.window.lease_stat.setText("Not Acquired")

    def _power_state_str(self):
        """Checking and displaying power status"""
        power_state = self._power_state()
        if power_state is None:
            return ''
        state_str = robot_state_proto.PowerState.MotorPowerState.Name(power_state)
        status = state_str[6:]
        self.window.power_stat.setText(status)  # Display status with QLabel object on GUI

    def _estop_str(self):
        """Checking and displaying Estop Status"""
        estop_status = '??'
        state = self.spot_core.robot_state
        if state:
            for estop_state in state.estop_states:
                if estop_state.type == estop_state.TYPE_SOFTWARE:
                    estop_status = estop_state.State.Name(estop_state.state)[6:]  # s/STATE_//
                    break
        self.window.e_stop_stat.setText(estop_status)  # Display status with QLabel object on GUI

    def _time_sync_str(self):
        """Checking and displaying time sync status"""
        if not self.spot_core._robot.time_sync:
            self.window.time_stat.setText('Time sync: (none)')
        if self.spot_core._robot.time_sync.stopped:
            status = 'STOPPED'
            exception = self.spot_core._robot.time_sync.thread_exception
            if exception:
                status = '{} Exception: {}'.format(status, exception)
        else:
            status = 'RUNNING'
        self.window.time_stat.setText(status)  # Display status with QLabel object on GUI

    def _battery_str(self):
        """Checking and displaying battery status"""
        if not self.spot_core.robot_state:
            return
        battery_state = self.spot_core.robot_state.battery_states[0]
        status = battery_state.Status.Name(battery_state.status)
        status = status[7:]  # get rid of STATUS_ prefix
        self.window.battery_stat.setText(status)  # Display battery status
        if battery_state.charge_percentage.value:
            # Display percentage with QProgressBar object
            self.window.battery_bar.setValue(int(battery_state.charge_percentage.value))
        else:
            self.window.battery_bar.setValue(0)
            self.window.battery_time.setText("0")
        if battery_state.estimated_runtime:
            self.window.battery_time.setText(
                "{}".format(secs_to_hms(battery_state.estimated_runtime.seconds)))  # Display runtime with QLabel object

    def _link_str(self):
        """Stream link data (position, velocity, load) on QTableWidget"""
        self.window.joint_table.setRowCount(12)
        row = 0
        # Loop through all joints and update appropriate data in each row
        for joint in self.spot_core.robot_state.kinematic_state.joint_states:
            self.window.joint_table.setItem(row, 0, QTableWidgetItem("{}".format(joint.name)))
            position = spot_r2d(joint.position)
            self.window.joint_table.setItem(row, 1, QTableWidgetItem("{}".format(position)))
            velocity = str(joint.velocity)
            self.window.joint_table.setItem(row, 2, QTableWidgetItem("{}".format(velocity[7:12])))
            load = str(joint.load)
            self.window.joint_table.setItem(row, 3, QTableWidgetItem("{}".format(load[7:12])))
            row += 1

    def _toggle_joint_record(self):
        """Start joint state recording"""
        self._record_joint = not self._record_joint
        if self._record_joint:
            self._record_jointstart = time.time()
            self._record_joint_row = 0

    def _toggle_state_record(self):
        """Start robot state recording"""
        self._record_state = not self._record_state
        if self._record_state:
            self._record_statestart = time.time()
            self._record_state_row = 0

    def _joint_record(self):
        """Record joint state and write to csv file in joint_state_log folder"""
        if self._record_joint:
            with open("joint_state_log/joint_states.csv", "a+", newline='') as file:
                if self._record_joint_row == 0:
                    file.write("time, ")
                    for joint in self.spot_core.robot_state.kinematic_state.joint_states:
                        file.write(joint.name + ", ")

                    file.write("\n")
                    file.write(str(time.time() - self._record_jointstart) + ", ")
                    for joint in self.spot_core.robot_state.kinematic_state.joint_states:
                        position = str(joint.position)
                        file.write(position[7:len(position) - 2] + ", ")

                    file.write("\n")
                else:
                    file.write(str(time.time() - self._record_jointstart) + ", ")
                    for joint in self.spot_core.robot_state.kinematic_state.joint_states:
                        position = str(joint.position)
                        file.write(position[7:len(position) - 2] + ", ")

                    file.write("\n")
                self._record_joint_row += 1

    def _state_record(self):
        """Record robot state and wrtie to csv file in state_log folder"""
        if self._record_state:
            with open("src/state_log/robot_state" + "{}".format(self._record_state_row) + ".json", "w") as file:
                file.write(MessageToJson(self.spot_core.robot_state, including_default_value_fields=True))
                self._record_state_row += 1

    def _current_position_str(self):
        """Stream position and rotation data in kinematic odometry frame to GUI"""
        if not self.spot_core.robot_state:
            return
        # kinematic odometry frame
        current_pos = self.spot_core.robot_state.kinematic_state.ko_tform_body.position
        self.window.pos_x.setText("{}".format(current_pos.x))
        self.window.pos_y.setText("{}".format(current_pos.y))
        self.window.pos_z.setText("{}".format(current_pos.z))
        current_rot = self.spot_core.robot_state.kinematic_state.ko_tform_body.rotation
        euler_zxy = geometry.to_euler_zxy(current_rot)
        self.window.pos_yaw.setText("{}".format(euler_zxy.yaw))
        self.window.pos_pitch.setText("{}".format(euler_zxy.pitch))
        self.window.pos_roll.setText("{}".format(euler_zxy.roll))

        # vision odometry frame
        current_pos_2 = self.spot_core.robot_state.kinematic_state.vo_tform_body.position
        self.window.pos_x_2.setText("{}".format(current_pos_2.x))
        self.window.pos_y_2.setText("{}".format(current_pos_2.y))
        self.window.pos_z_2.setText("{}".format(current_pos_2.z))
        current_rot_2 = self.spot_core.robot_state.kinematic_state.vo_tform_body.rotation
        euler_zxy_2 = geometry.to_euler_zxy(current_rot_2)
        self.window.pos_yaw_2.setText("{}".format(euler_zxy_2.yaw))
        self.window.pos_pitch_2.setText("{}".format(euler_zxy_2.pitch))
        self.window.pos_roll_2.setText("{}".format(euler_zxy_2.roll))

    def _status_update(self):
        """Update all status at each singal call"""
        self._lease_str()
        self._battery_str()
        self._estop_str()
        self._power_state_str()
        self._time_sync_str()
        self._link_str()
        self.camera.cv_camera_update()
        self._joint_record()
        self._state_record()
        self._current_position_str()

    def _log_update(self, msg):
        """Update log at each signal call"""
        self.window.qtlog.append(msg)

    class DriveInterface(QDialog):
        """A dialog prompt in main GUI to allow keyboard control of Spot using same keybindings from wasd.py"""
        key = Signal(int)

        def __init__(self):
            super().__init__()
            # Organize QDialog elements
            self.setWindowTitle("Key Input")
            self.text = QTextBrowser()
            self.text.setPlainText(
                "Commands: [TAB]: quit\n\
                [T]: Time-sync, [SPACE]: Estop, [P]: Power\n\
                [v]: Sit, [f]: Stand, [r]: Self-right\n\
                [wasd]: Directional strafing\n\
                [qe]: Turning"
            )
            self.quit = QPushButton("Quit")
            self.layout = QVBoxLayout()
            self.layout.addWidget(self.text)
            self.layout.addWidget(self.quit)
            self.setLayout(self.layout)
            self.quit.clicked.connect(self.close)
            self.resize(300, 200)
            self.show()

        def keyPressEvent(self, event):
            """Overide QObject keyPressEvent to emit signal at each key press"""
            if event.type() == QEvent.KeyPress:
                self.key.emit(event.key())

    def _toggle_key(self):
        """Initialize a Qdialog to start keyboard command of Spot"""
        self.keyboard = self.DriveInterface()
        self.keyboard.key.connect(self._key_command)

    @Slot(int)
    def _key_command(self, key_input):
        """Match a key input with command"""
        command = self._command_dictionary[key_input]
        command()


def _setup_logging():
    """Log to file at debug level, and log to console at INFO or DEBUG (if verbose).

    """
    LOGGER.setLevel(logging.DEBUG)
    log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

    # Save log messages to file spotGUI.log for later debugging.
    file_handler = logging.FileHandler('spotGUI.log')
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(log_formatter)
    LOGGER.addHandler(file_handler)


def spot_r2d(data):
    """Function to parse joint position data in kinematic_state and return them in degrees"""
    radian = str(data)
    try:
        degree = str(math.degrees(float(radian[7:len(radian) - 2])))
        degree = degree[:5]
    except ValueError:
        print(radian[7:len(radian) - 2])
        degree = 0
    return degree


def main():
    """Spot Qt GUI Interface"""

    _setup_logging()
    spot_interface = SpotInterface()
    spot_interface.app.exec_()

    return True


if __name__ == "__main__":
    if not main():
        sys.exit(1)
