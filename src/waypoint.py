from collections import OrderedDict
from PySide2.QtCore import QThread
import math
import time


class WayPointExecution(QThread):
    def __init__(self, waypointcontrol):
        QThread.__init__(self)
        self.waypoint_control = waypointcontrol

    def run(self):
        for name, pose in self.waypoint_control.waypoints().items():
            current_pose = self.waypoint_control.spot_core.robot_state.kinematic_state.ko_tform_body.position
            distance = math.sqrt((pose[0] - current_pose.x) ** 2 + (pose[1] - current_pose.y) ** 2)
            time_secs = distance * 4
            start = time.time()
            self.waypoint_control.spot_core._robot.logger.info("Waypoint: {}, X: {}, Y: {}, Heading: {}, Duration: {}"
                                                               .format(name, pose[0], pose[1], pose[2], time_secs))
            self.waypoint_control.spot_core.go_to_waypoint(desc=name, time_secs=time_secs, goal_x=pose[0], goal_y=pose[1],
                                           goal_heading=pose[2])
            while (time.time() - start) < time_secs:
                time.sleep(0.1)
                if not self.waypoint_control.execution():
                    self.waypoint_control.spot_core._robot.logger.info("Waypoint Execution Stopped")
                    return


class WayPointControl(object):
    def __init__(self, tablewidget, spot_core):
        self.waypoint_table = tablewidget
        self._waypoints = WayPointQueue()
        self.spot_core = spot_core
        self._execute = False
        self.subthread = None

    def stop_execution(self):
        self._execute = False

    def execution(self):
        return self._execute

    def add_row(self):
        row = self.waypoint_table.rowCount()
        self.waypoint_table.setRowCount(row + 1)

    def save(self):
        row = self.waypoint_table.rowCount()
        for i in range(row):
            name = self.waypoint_table.item(i, 0).text()
            x = float(self.waypoint_table.item(i, 1).text())
            y = float(self.waypoint_table.item(i, 2).text())
            heading = float(self.waypoint_table.item(i, 3).text())
            self._waypoints.add_point(name, (x, y, heading))

    def waypoints(self):
        return self._waypoints.get()

    def go_to_waypoint(self):
        self._execute = True
        self.subthread = WayPointExecution(self)
        self.subthread.start()


class WayPointQueue:
    def __init__(self):
        self._waypoints = OrderedDict()

    def get(self):
        return self._waypoints

    def add_point(self, name, waypoint):
        self._waypoints[name] = waypoint

    def remove_point(self, name):
        self._waypoints.pop(name)

    def remove_last_point(self):
        self._waypoints.popitem()
