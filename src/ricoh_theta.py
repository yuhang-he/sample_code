import requests
from requests.auth import HTTPDigestAuth
from PySide2.QtWidgets import QButtonGroup, QTableWidgetItem
import json
import pprint


class RicohThetaControl(object):

    def __init__(self, window):
        self._image_mode = True
        self._video_mode = False
        self._plugin = None
        self._ip_address = None
        # TODO: Ricoh Theta ID and password (Edit)
        self._theta_id = "THETAYL00196071"
        self._theta_pwd = "TeslaSpot00196071"
        self._osc_path = None
        self._cmd_id = None
        self._resp = None
        self.print = pprint.PrettyPrinter()

        self._window = window
        self._window.rtv_connect.clicked.connect(self.connect)
        self._window.rtv_info.clicked.connect(self.info)
        self._window.rtv_state.clicked.connect(self.state)
        self._window.rtv_cmd_stat.clicked.connect(self.cmd_status)
        self._window.rtv_cap.clicked.connect(self.take_pic)
        self._window.rtv_start_rec.clicked.connect(self.start_rec)
        self._window.rtv_stop_rec.clicked.connect(self.stop_rec)

        self._cap_mode = QButtonGroup()
        self._cap_mode.addButton(self._window.rtv_image)
        self._cap_mode.addButton(self._window.rtv_video)
        self._cap_mode.buttonClicked.connect(self.toggle_capture_mode)

        self._window.rtv_list_plugins.clicked.connect(self.list_plugins)
        self._window.rtv_select_plugin.activated.connect(self.select_plugin)
        self._window.rtv_start_plugin.clicked.connect(self.start_plugin)
        self._window.rtv_stop_plugin.clicked.connect(self.stop_plugin)
        self.plugin_dict = {}

        self._window.rtv_list_files.clicked.connect(self.list_files)

    def _get(self, osc_command, response=True):
        """General command to request information from Theta using Google's Open Spherical Camera API

        Args:
            osc_command: Richo Theta Web API command header (commands/status or commands/execute)
            response: Flag for whether to print camera response

        Returns:Richo Theta response

        """
        url = self._osc_path + osc_command
        self._resp = requests.get(url, auth=(HTTPDigestAuth(self._theta_id, self._theta_pwd)))
        if response:
            self._window.qtlog.append(self.print.pformat(self._resp.json()))

    def _post(self, osc_command, param=None, response=True):
        """eneral command to post commands to Theta using Google's Open Spherical Camera API

        Args:
            osc_command: Richo Theta Web API command header (commands/status or commands/execute)
            param: specific Web API command and its parameter in JSON format
            response: lag for whether to print camera response

        Returns: Richo Theta response

        """
        url = self._osc_path + osc_command
        self._resp = requests.post(url, json=param, auth=(HTTPDigestAuth(self._theta_id, self._theta_pwd)))
        if response:
            self._window.qtlog.append(self.print.pformat(self._resp.json()))

    def _cmd_id_update(self):
        """Work in progress (update Theta command ID)

        Returns: Theta command ID

        """
        resp = self._resp.json()
        if "id" in resp:
            self._cmd_id = resp["id"]

    def connect(self):
        """Setup HTTP address to connect to Theta

        Returns: update HTTP address to self._osc_path

        """
        self._ip_address = self._window.rtv_ip.text()
        self._osc_path = "http://" + self._ip_address + "/osc/"

    def info(self):
        """Request camera info from Theta

        Returns:

        """
        osc_command = "info"
        self._get(osc_command)

    def state(self):
        """Request camera state from Theta

        Returns:

        """
        osc_command = "state"
        self._post(osc_command)

    def cmd_status(self, response=True):
        """Check status of command

        Args:
            response: Flag for whether to print camera response

        Returns:

        """
        # TODO: work in progress
        osc_command = "commands/status"
        self._cmd_id_update()
        param = {"id": self._cmd_id}
        self._window.qtlog.append("{}".format(self._cmd_id))
        self._post(osc_command, param, response)

    def toggle_capture_mode(self):
        """Toggle between camera shoot and video record mode

        Returns:

        """
        self.cmd_status(response=False)
        resp = self._resp.json()
        if resp["state"] == "inProgress":
            self._window.qtlog.append("{} is still in progress".format(resp["name"]))
        else:
            if self._window.rtv_image.isChecked():
                self._image_mode = True
                self._video_mode = False
            else:
                self._image_mode = False
                self._video_mode = True

            if self._image_mode:
                mode = "image"
            else:
                mode = "video"
            osc_command = "commands/execute"
            param = {"name": "camera.setOptions",
                     "parameters": {
                         "options": {
                             "captureMode": mode
                         }
                     }}
            self._post(osc_command, param)

    def take_pic(self):
        """Capture 360 picture

        Returns:

        """
        osc_command = "commands/execute"
        param = {"name": "camera.takePicture"}
        self._post(osc_command, param)

    def start_rec(self):
        """Start recording 360 video

        Returns:

        """
        osc_command = "commands/execute"
        param = {"name": "camera.startCapture"}
        self._post(osc_command, param)

    def stop_rec(self):
        """Stop recording 360 video

        Returns:

        """
        osc_command = "commands/execute"
        param = {"name": "camera.stopCapture"}
        self._post(osc_command, param)

    def list_plugins(self):
        """List available plugins (directly update them to Qt GUI)

        Returns:

        """
        osc_command = "commands/execute"
        param = {"name": "camera._listPlugins"}
        self._post(osc_command, param, response=True)
        plugins = self._resp.json()
        index = 1
        for plugin in plugins["results"]["plugins"]:
            self._window.rtv_select_plugin.insertItem(index, plugin["pluginName"])
            self.plugin_dict[plugin["pluginName"]] = plugin["packageName"]
            index += 1

    def select_plugin(self):
        """Set Theta plugin based on Qt GUI selection

        Returns:

        """
        plugin_name = self._window.rtv_select_plugin.currentText()
        osc_command = "commands/execute"
        param = {"name": "camera._setPlugin",
                 "parameters": {
                     "packageName": self.plugin_dict[plugin_name],
                     "boot": True
                 }}
        self._post(osc_command, param)

    def start_plugin(self):
        """Start plugin previously selected

        Returns:

        """
        osc_command = "commands/execute"
        param = {"name": "camera._pluginControl",
                 "parameters": {
                     "action": "boot"
                 }}
        self._post(osc_command, param)

    def stop_plugin(self):
        """Stop plugin (however, if plugin uses Theta Camera API, then Web API will no longer work)

        Returns:

        """
        osc_command = "commands/execute"
        param = {"name": "camera._pluginControl",
                 "parameters": {
                     "action": "finish"
                 }}
        self._post(osc_command, param, response=True)

    def list_files(self):
        """List picture and video files stored on Theta. Update list to Qt GUI

        Returns:

        """
        osc_command = "commands/execute"
        param = {"name": "camera.listFiles",
                 "parameters": {
                     "fileType": "all",
                     "entryCount": 100,
                     "maxThumbSize": 0,
                     "_detail": "false"
                 }}
        self._post(osc_command, param, response=True)
        files = self._resp.json()
        self._window.file_table.setRowCount(files["results"]["totalEntries"])
        row = 0
        for file in files["results"]["entries"]:
            self._window.file_table.setItem(row, 0, QTableWidgetItem("{}".format(file["name"])))
            self._window.file_table.setItem(row, 1, QTableWidgetItem("{}".format(file["fileUrl"])))
            self._window.file_table.setItem(row, 2, QTableWidgetItem("{}".format(file["size"])))
            self._window.file_table.setItem(row, 3, QTableWidgetItem("{}".format(file["dateTime"])))
            row += 1
