from PySide2.QtWidgets import QButtonGroup
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2

class MobilityParams(object):

    def __init__(self, window, spot_core):
        self._window = window
        self._spot_core = spot_core
        self.walking_gait = QButtonGroup()
        self.walking_gait.addButton(self._window.gait_auto)
        self.walking_gait.addButton(self._window.gait_trot)
        self.walking_gait.addButton(self._window.gait_auto_trot)
        self.walking_gait.addButton(self._window.gait_crawl)
        self.walking_gait.addButton(self._window.gait_amble)
        self.walking_gait.addButton(self._window.gait_auto_amble)
        self.walking_gait.addButton(self._window.gait_jog)
        self.walking_gait.addButton(self._window.gait_hop)

        self.swing_height = QButtonGroup()
        self.swing_height.addButton(self._window.swing_h_low)
        self.swing_height.addButton(self._window.swing_h_med)
        self.swing_height.addButton(self._window.swing_h_high)

        # Connect each setting their appropriate function call
        self._window.enable_param.clicked.connect(self.enable_params)
        self.walking_gait.buttonClicked.connect(self.gait_update)
        self.swing_height.buttonClicked.connect(self.swing_height_update)
        self._window.stair.clicked.connect(self.stair_update)
        self._window.walk_height.valueChanged.connect(self.body_pose_update)
        self._window.avoidance.valueChanged.connect(self.obstacle_params_update)
        self._window.degraded_perc.clicked.connect(self.obstacle_params_update)
        self._window.disable_f_obs.clicked.connect(self.obstacle_params_update)
        self._window.disable_b_obs.clicked.connect(self.obstacle_params_update)
        self._window.disable_f_cons.clicked.connect(self.obstacle_params_update)
        self._window.speed.valueChanged.connect(self.speed_update)
        self._window.pose_yaw.valueChanged.connect(self.body_pose_update)
        self._window.pose_roll.valueChanged.connect(self.body_pose_update)
        self._window.pose_pitch.valueChanged.connect(self.body_pose_update)

        # Match button selection with locomotion hint enumerated in spot/robot_command.proto
        self._gait_dictionary = {
            "Auto": spot_command_pb2.HINT_AUTO,
            "Auto Trot": spot_command_pb2.HINT_SPEED_SELECT_TROT,
            "Trot": spot_command_pb2.HINT_TROT,
            "Crawl": spot_command_pb2.HINT_CRAWL,
            "Amble": spot_command_pb2.HINT_AMBLE,
            "Auto Amble": spot_command_pb2.HINT_SPEED_SELECT_AMBLE,
            "Jog": spot_command_pb2.HINT_JOG,
            "Hop": spot_command_pb2.HINT_HOP,
        }
        # Match button selection with swing_height enumerated in spot/robot_command.proto
        self._swing_dict = {
            "Low": spot_command_pb2.SWING_HEIGHT_LOW,
            "Medium": spot_command_pb2.SWING_HEIGHT_MEDIUM,
            "High": spot_command_pb2.SWING_HEIGHT_HIGH,
        }

    def enable_params(self):
        """Enable custom mobility parameter settings through GUI"""
        self._spot_core.enable_mobility_params(self._window.enable_param.isChecked())

    def speed_update(self):
        """Update Spot speed parameter through GUI"""
        self._spot_core.speed_update(self._window.speed.value())

    def gait_update(self):
        """Update Spot gait parameter through GUI"""
        self._spot_core.locomotion_hint_update(self._gait_dictionary[self.walking_gait.checkedButton().text()])

    def swing_height_update(self):
        """Update Spot leg swing height through GUI"""
        self._spot_core.swing_height_update(self._swing_dict[self.swing_height.checkedButton().text()])

    def stair_update(self):
        """Check stair mode through GUI"""
        self._spot_core.stair_update(self._window.stair.isChecked())

    def body_pose_update(self):
        """Update body_pose with Body Control settings entered in GUI"""
        try:
            height = self._window.walk_height.value()
            if height > 0.12 or height < -0.17:
                self._spot_core._robot.logger.info("The height is outside of range")
            else:
                self._spot_core.body_pose_update(body_height=height,
                                                 yaw=float(self._window.pose_yaw.value()),
                                                 roll=float(self._window.pose_roll.value()),
                                                 pitch=float(self._window.pose_pitch.value()))
        except TypeError:
            self._spot_core._robot.logger.info("The height entered is not acceptable")

    def obstacle_params_update(self):
        """Update obstacle_params with Perception settings"""
        self._spot_core.\
            obstacle_params_update(obstacle_avoidance_padding=self._window.avoidance.value(),
                                   allow_degraded_perception=self._window.degraded_perc.isChecked(),
                                   disable_vision_foot_obstacle_avoidance=self._window.disable_f_obs.isChecked(),
                                   disable_vision_foot_constraint_avoidance=self._window.disable_f_cons.isChecked(),
                                   disable_vision_body_obstacle_avoidance=self._window.disable_b_obs.isChecked())
