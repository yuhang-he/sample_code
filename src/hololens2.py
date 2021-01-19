import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web
import asyncio
import json
import math
from json import JSONDecodeError
from PySide2.QtCore import QThread
from google.protobuf.json_format import MessageToJson


class WSHandler(tornado.websocket.WebSocketHandler):
    def initialize(self, window, handlers, websocket):
        self._window = window
        self.connection = False
        self._handlers = handlers
        self.ws = websocket

    def open(self):
        self._window.qtlog.append("Connection Opened")
        self.connection = True
        self.set_nodelay(True)
        self._handlers.append(self)

    def on_message(self, message):
        """Convert HoloLens inputs into Spot commands

        Parse waypoint x, y, z, and direction in radians from JSON

        Args:
            message: string message from websocket

        Returns:

        """
        try:
            command = self.ws.command_dictionary[message]
            command()
        except KeyError:
            try:
                data = json.loads(message)
                x = data["position"]["x"]
                y = data["position"]["y"]
                z = data["position"]["z"]
                heading = self.ws.quat_to_euler(
                    (data["rotation"]["x"], data["rotation"]["y"], data["rotation"]["z"], data["rotation"]["w"]))[
                              2] + math.pi / 2

                robot_state = self.ws._spot_core.robot_state.kinematic_state.ko_tform_body
                current_pose = robot_state.position
                current_angle = self.ws.quat_to_euler((robot_state.rotation.x, robot_state.rotation.y,
                                                       robot_state.rotation.z, robot_state.rotation.w))[2]
                distance = math.sqrt((x - current_pose.x) ** 2 + (y - current_pose.y) ** 2)
                time_secs = distance * 4
                self.ws._spot_core.add_message(
                    "x: {} y: {} z: {} heading: {} current angle: {} during: {}s".format(x, y, z, heading,
                                                                                         current_angle, time_secs))
                self.ws._spot_core.go_to_waypoint(desc="HoloLens Control", time_secs=time_secs, goal_x=x,
                                                  goal_y=y,
                                                  goal_heading=heading)
            except JSONDecodeError:
                self.ws._spot_core.add_message("JSON Cannot parse: " + message)

    def on_close(self):
        self.connection = False
        self._window.qtlog.append("Connection Closed")

    def check_origin(self, origin):
        return True


class WebSocketThread(QThread):
    """A separate thread to run websocket server to not block the main GUI thread"""

    def __init__(self, tornado_ws):
        QThread.__init__(self)
        self.tornado_ws = tornado_ws

    def run(self):
        asyncio.set_event_loop(asyncio.new_event_loop())
        self.tornado_ws.start_server()


class HoloWebSocket(object):
    """Integrate Tornado websocket library with Spot GUI to control pan tilt unit"""

    def __init__(self, spot_core, window):
        self._spot_core = spot_core
        self._window = window

        # IP Address and port number of WebSocket
        self.address = "127.0.0.1"
        self.port = 8080
        self.websocket_thread = None
        self.ioloop = None
        self.http_server = None
        self.initial = True
        self.handlers = []

        self.robot_state = None
        self.command_list = None

        self._window.holo_connect.clicked.connect(self.ws_connect)
        self._window.holo_disconnect.clicked.connect(self.ws_disconnect)
        self._window.holo_disconnect.setEnabled(False)
        self._spot_core.stat_update.connect(self.transmit)

        self.application = tornado.web.Application(
            [(r'/', WSHandler, {"window": window, "handlers": self.handlers, "websocket": self})], )

        self.command_dictionary = {
            "Time Sync": self._spot_core.toggle_time_sync,
            "Estop": self._spot_core.toggle_estop,
            "Self Right": self._spot_core.self_right,
            "Toggle Power": self._spot_core.toggle_power,
            "Sit": self._spot_core.sit,
            "Stand": self._spot_core.stand,
            "Forward": self._spot_core.move_forward,
            "Backward": self._spot_core.move_backward,
            "Left": self._spot_core.strafe_left,
            "Right": self._spot_core.strafe_right,
            "Turn Left": self._spot_core.turn_left,
            "Turn Right": self._spot_core.turn_right,
        }

        @staticmethod
        def quat_to_euler(q):
            """Convert a quaternion to xyz Euler angles"""
            roll = math.atan2(2 * q[3] * q[0] + q[1] * q[2], 1 - 2 * q[0] ** 2 + 2 * q[1] ** 2)
            pitch = math.atan2(2 * q[1] * q[3] - 2 * q[0] * q[2], 1 - 2 * q[1] ** 2 - 2 * q[2] ** 2)
            yaw = math.atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], 1 - 2 * q[1] ** 2 - 2 * q[2] ** 2)
            return roll, pitch, yaw

    def transmit(self):
        """Up on change to control elements, transmit new commands through websocket

        Returns:

        """
        self.robot_state = MessageToJson(self._spot_core.robot_state, including_default_value_fields=True)

        for handler in self.handlers:
            if handler.connection:
                handler.write_message(self.robot_state)

    def ws_connect(self):
        """Start separate thread for websocket server. Enable and disable connection buttons

        Returns:

        """
        self.websocket_thread = WebSocketThread(self)
        self.websocket_thread.start()
        self._window.holo_connect.setEnabled(False)
        self._window.holo_disconnect.setEnabled(True)

    def ws_disconnect(self):
        """Close websocket server and stop thread. Enable and disable connection buttons

        Returns:

        """
        self.ioloop.add_callback(self.ioloop.stop)
        self.http_server.stop()
        self._window.qtlog.append("Server Closed")
        self._window.holo_connect.setEnabled(True)
        self._window.holo_disconnect.setEnabled(False)

    def start_server(self):
        """Start websocket server at specified address and port from a separate thread

        Returns:

        """
        try:
            self.address = self._window.holo_address.text()
            self.port = int(self._window.holo_port.text())
        except ValueError:
            self._window.qtlog.append("Invalid address or port number")
        self.http_server = tornado.httpserver.HTTPServer(self.application)
        self.http_server.listen(port=self.port, address=self.address)
        self._window.qtlog.append("Start server at {}: {}".format(self.address, self.port))
        self.ioloop = tornado.ioloop.IOLoop.current()
        tornado.ioloop.IOLoop.current().start()
