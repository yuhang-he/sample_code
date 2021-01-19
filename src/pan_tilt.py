import tornado.httpserver
import tornado.websocket
import tornado.ioloop
import tornado.web
import asyncio
import json
from PySide2.QtCore import QThread


class WSHandler(tornado.websocket.WebSocketHandler):
    def initialize(self, window, handlers):
        self._window = window
        self.connection = False
        self._handlers = handlers

    def open(self):
        self._window.qtlog.append("Connection Opened")
        self.connection = True
        self.set_nodelay(True)
        self._handlers.append(self)

    def on_message(self, message):
        self._window.qtlog.append(message)

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


class ArduinoWebSocket(object):
    """Integrate Tornado websocket library with Spot GUI to control pan tilt unit"""

    def __init__(self, window):
        self._window = window

        # IP Address and port number of WebSocket
        self.address = "127.0.0.1"
        self.port = 8080
        self.websocket_thread = None
        self.ioloop = None
        self.http_server = None
        self.initial = True
        self.handlers = []

        self.laser = 0
        self.servo1 = 0
        self.servo2 = 0
        self.servo3 = 0
        self.command = None
        self.imu_data = None

        self._window.pan_tilt_server.clicked.connect(self.ws_connect)
        self._window.pan_tilt_disconnect.clicked.connect(self.ws_disconnect)
        self._window.pan_tilt_disconnect.setEnabled(False)
        self._window.laser_toggle.toggled.connect(self.transmit)
        self._window.servo1.valueChanged.connect(self.transmit)
        self._window.servo2.valueChanged.connect(self.transmit)
        self._window.servo3.valueChanged.connect(self.transmit)

        self.application = tornado.web.Application([(r'/', WSHandler, {"window": window, "handlers": self.handlers})], )

    def transmit(self):
        """Up on change to control elements, transmit new commands through websocket

        Returns:

        """
        if self._window.laser_toggle.isChecked():
            self.laser = 1
        else:
            self.laser = 0
        self.servo1 = self._window.servo1.value()
        self.servo2 = self._window.servo2.value()
        self.servo3 = self._window.servo3.value()

        self.command = {"laser": self.laser, "servo1": self.servo1, "servo2": self.servo2, "servo3": self.servo3}
        message = json.dumps(self.command, indent=4)

        for handler in self.handlers:
            if handler.connection:
                handler.write_message(message)
        # self._window.qtlog.append(message)

    def ws_connect(self):
        """Start separate thread for websocket server. Enable and disable connection buttons

        Returns:

        """
        self.websocket_thread = WebSocketThread(self)
        self.websocket_thread.start()
        self._window.pan_tilt_server.setEnabled(False)
        self._window.pan_tilt_disconnect.setEnabled(True)

    def ws_disconnect(self):
        """Close websocket server and stop thread. Enable and disable connection buttons

        Returns:

        """
        self.ioloop.add_callback(self.ioloop.stop)
        self.http_server.stop()
        self._window.qtlog.append("Server Closed")
        self._window.pan_tilt_server.setEnabled(True)
        self._window.pan_tilt_disconnect.setEnabled(False)

    def start_server(self):
        """Start websocket server at specified address and port from a separate thread

        Returns:

        """
        try:
            self.address = self._window.pan_tilt_address.text()
            self.port = int(self._window.pan_tilt_port.text())
        except ValueError:
            self._window.qtlog.append("Invalid address or port number")
        self.http_server = tornado.httpserver.HTTPServer(self.application)
        self.http_server.listen(port=self.port, address=self.address)
        self._window.qtlog.append("Start server at {}: {}".format(self.address, self.port))
        self.ioloop = tornado.ioloop.IOLoop.current()
        tornado.ioloop.IOLoop.current().start()
