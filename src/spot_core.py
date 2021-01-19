from __future__ import print_function
import logging
import time

from bosdyn import geometry
from bosdyn.api import geometry_pb2, trajectory_pb2
import bosdyn.api.power_pb2 as power_service_proto
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
from bosdyn.client import ResponseError, RpcError
from bosdyn.client.async_tasks import AsyncGRPCTask, AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.image import ImageClient
from bosdyn.client.robot_state import RobotStateClient

from PySide2.QtCore import QThread, Signal, Slot, QObject

LOGGER = logging.getLogger()

VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 0.6  # seconds


def _grpc_or_log(desc, thunk):
    """Try to send request through gRPC. On failure, log error response

    Args:
        desc: Description of request
        thunk: gRPC request

    Returns:

    """
    try:
        return thunk()
    except (ResponseError, RpcError) as err:
        LOGGER.error("Failed %s: %s", desc, err)


class AsyncMetrics(AsyncPeriodicQuery):
    """Grab robot metrics every few seconds."""

    def __init__(self, robot_state_client):
        super(AsyncMetrics, self).__init__("robot_metrics", robot_state_client, LOGGER,
                                           period_sec=5.0)

    def _start_query(self):
        return self._client.get_robot_metrics_async()


class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state"""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__("robot_state", robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()


class AsyncImageCapture(AsyncGRPCTask):
    """Grab camera images from the robot."""

    def __init__(self, robot):
        super(AsyncImageCapture, self).__init__()
        self._image_client = robot.ensure_client(ImageClient.default_service_name)
        self._video_mode = False
        self._should_take_image = False
        self._image_data = [None] * 5
        self.image_sources = []

    def toggle_video_mode(self):
        """Toggle continuous image capture."""
        self._video_mode = not self._video_mode

    def take_image(self):
        """Request a one-shot image."""
        self._should_take_image = True

    def _start_query(self):
        self._should_take_image = False
        return self._image_client.get_image_from_sources_async(self.image_sources)

    def _should_query(self, now_sec):  # pylint: disable=unused-argument
        return self._video_mode or self._should_take_image

    def _handle_result(self, result):
        """Store raw image data in _image_data list"""
        image_num = 0
        for img_response in result:
            self._image_data[image_num] = img_response.shot.image.data
            image_num += 1

    def _handle_error(self, exception):
        LOGGER.exception("Failure getting image: %s", exception)


class AsyncUpdateThread(QThread):
    """Create a QThread to handle looping async_task update"""
    stat = Signal()

    def __init__(self, async_tasks):
        QThread.__init__(self)
        self._async_tasks = async_tasks
        self.update = False

    def run(self):
        while self.update:
            self._async_tasks.update()
            time.sleep(0.01)  # Slow down update rate to reduce impact on performace
            self.stat.emit()


class GUIHandler(logging.Handler):
    """logging handler which puts messages into GUI log"""

    def __init__(self, spotinterface):
        super(GUIHandler, self).__init__()
        self._spot_interface = spotinterface

    def emit(self, record):
        msg = record.getMessage()
        msg = msg.replace('\n', ' ').replace('\r', '')
        self._spot_interface.add_message('{:s} {:s}'.format(record.levelname, msg))


class SpotCore(QObject):
    """Heavily modified wasd.py script to work with Qt GUI"""
    log = Signal(object)
    stat_update = Signal()

    def __init__(self, robot):
        super(SpotCore, self).__init__()

        self._robot = robot
        # Create clients -- do not use the for communication yet.
        self._lease_client = robot.ensure_client(LeaseClient.default_service_name)
        self._estop_client = robot.ensure_client(EstopClient.default_service_name)
        self._estop_endpoint = EstopEndpoint(self._estop_client, 'GUI', 9.0)
        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        self._robot_metrics_task = AsyncMetrics(self._robot_state_client)
        self._robot_state_task = AsyncRobotState(self._robot_state_client)
        self._image_task = AsyncImageCapture(robot)
        self._async_tasks = AsyncTasks(
            [self._robot_metrics_task, self._robot_state_task, self._image_task])

        self._async_update_thread = None

        self._estop_keepalive = None
        self._lease_keepalive = None
        self._exit_check = None

        self._robot_id = None
        self._lease = None

        # parameters that can be set in mobility parameter
        self._base_speed = 0.5  # m/s
        self._custom_params = False
        self._mobility_params = None
        self._obstacle_params = None
        self._locomotion_hint = spot_command_pb2.HINT_AUTO_TROT
        self._swing_height = spot_command_pb2.SWING_HEIGHT_MEDIUM
        self._stair = False
        self._body_height = 0.0
        self._pose_yaw = 0.0
        self._pose_roll = 0.0
        self._pose_pitch = 0.0
        self._allow_degraded_perception = False

        gui_handler = GUIHandler(self)
        gui_handler.setLevel(logging.INFO)
        LOGGER.addHandler(gui_handler)

    def control(self):
        """Begin communication with the robot in control mode."""
        try:
            self._estop_endpoint.role = "PDB_rooted"
            self._robot.logger.info("Acuquiring robot ID, lease, and setup Estop")
            self._robot_id = self._robot.get_id()
            self._lease = self._lease_client.take()
            # self._lease = self._lease_client.acquire()
            self._lease_keepalive = LeaseKeepAlive(self._lease_client)
            self._estop_endpoint.force_simple_setup()  # Set this endpoint as the robot's sole estop.
            self.run()
            self._robot.logger.info("Start Successful")
        except (ResponseError, RpcError) as err:
            LOGGER.error("Failed to initialize robot communication: %s", err)
            return False

    def observe(self):
        """Begin communication with the robot in observe mode."""
        try:
            self._robot.logger.info("Observing Robot State")
            self._robot_id = self._robot.get_id()
            # self._estop_endpoint.role = "observer"
            #
            # active_config = self._estop_client.get_config()
            # new_endpoint = active_config.endpoints.add()
            # new_endpoint.CopyFrom(self._estop_endpoint.to_proto())
            #
            # active_config = self._estop_client.set_config(active_config, active_config.unique_id)
            # self.add_message("{}, {}".format(active_config.endpoints[0].name, active_config.endpoints[0].name))
            # self._estop_endpoint.register(active_config.unique_id)
            # active_config = self._estop_client.get_config()
            self.run()
            self._robot.logger.info("Start Successful")
        # except ResponseError:
        #     self._estop_endpoint.force_simple_setup()
        except (ResponseError, RpcError) as err:
            LOGGER.error("Failed to initialize robot communication: %s", err)
            return False

    def shutdown(self):
        """Release control of robot as gracefully as possible."""
        self._robot.logger.info("Robot Shutdown: Stop time sync, stop Estop keepalive, return lease")
        self._async_update_thread.update = False
        if self._robot.time_sync:
            self._robot.time_sync.stop()
        if self._estop_keepalive:
            _grpc_or_log("stopping estop", self._estop_keepalive.stop)
            self._estop_keepalive = None
        if self._lease_keepalive:
            _grpc_or_log("stopping lease", self._lease_keepalive.shutdown)
            # self._lease_keepalive = None
        if self._lease:
            _grpc_or_log("returning lease", lambda: self._lease_client.return_lease(self._lease))
            self._lease = None
        self._robot.logger.info("Shutdown Successful")

    def __del__(self):
        self.shutdown()

    def add_message(self, msg_text):
        """Display the given message string to the user in the GUI log."""
        self.log.emit("{}".format(msg_text))

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self._robot_state_task.proto

    @property
    def robot_metrics(self):
        """Get latest robot metrics proto."""
        return self._robot_metrics_task.proto

    def run(self):
        """Create AsyncUpdateThread to receive state updates from Spot"""
        self._robot.logger.info("Start Robot State Thread")
        self._async_update_thread = AsyncUpdateThread(self._async_tasks)
        self._async_update_thread.update = True
        self._async_update_thread.stat.connect(self.status_update)
        self._async_update_thread.start()
        self._robot.logger.info("Robot State Thread Started")
        time.sleep(0.5)

    def _try_grpc(self, desc, thunk):
        """Try sending the commands through gRPC to Spot"""
        try:
            return thunk()
        except (ResponseError, RpcError) as err:
            self.add_message("Failed {}: {}".format(desc, err))
            return None

    def safe_power_off(self):
        """Invoke Spot's safe power off command"""
        self._robot.logger.info("Safe Power Off")
        self._sit()
        self._safe_power_off()
        self._robot.logger.info("Power Off Successful")

    def toggle_time_sync(self):
        """Toggle Spot time sync"""
        self._robot.logger.info("Toggling Time Sync")
        if self._robot.time_sync.stopped:
            self._robot.time_sync.start()
            self._robot.logger.info("Time Sync Started")
        else:
            self._robot.time_sync.stop()
            self._robot.logger.info("Time Sync Stopped")

    def toggle_estop(self):
        """toggle estop on/off"""
        self._robot.logger.info("Toggling Estop")
        if not self._estop_keepalive:
            self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
            self._robot.logger.info("Estop Started")
        else:
            self._try_grpc("stopping estop", self._estop_keepalive.stop)
            self._estop_keepalive.shutdown()
            self._estop_keepalive = None
            self._robot.logger.info("Estop Stopped")

    def _start_robot_command(self, desc, command_proto, end_time_secs=None):
        """Start robot command"""
        def _start_command():
            self._robot_command_client.robot_command(lease=None, command=command_proto,
                                                     end_time_secs=end_time_secs)
        self._try_grpc(desc, _start_command)

    def self_right(self):
        self._start_robot_command('self_right', RobotCommandBuilder.selfright_command())
        self._robot.logger.info("Self Right")

    def sit(self):
        self._start_robot_command('sit', RobotCommandBuilder.sit_command())
        self._robot.logger.info("Sit")

    def stand(self):
        if self._custom_params:
            params = self._mobility_params
        else:
            params = None
        self._start_robot_command('stand', RobotCommandBuilder.stand_command(params=params))
        self._robot.logger.info("Stand")

    def move_forward(self):
        self._velocity_cmd_helper('move_forward', v_x=self._base_speed)
        self._robot.logger.debug("Move_forward")

    def move_backward(self):
        self._velocity_cmd_helper('move_backward', v_x=-self._base_speed)
        self._robot.logger.debug("Move_backward")

    def strafe_left(self):
        self._velocity_cmd_helper('strafe_left', v_y=self._base_speed)
        self._robot.logger.debug("Strafe Left")

    def strafe_right(self):
        self._velocity_cmd_helper('strafe_right', v_y=-self._base_speed)
        self._robot.logger.debug("Strafe Right")

    def turn_left(self):
        self._velocity_cmd_helper('turn_left', v_rot=VELOCITY_BASE_ANGULAR)
        self._robot.logger.debug("Turn Left")

    def turn_right(self):
        self._velocity_cmd_helper('turn_right', v_rot=-VELOCITY_BASE_ANGULAR)
        self._robot.logger.debug("Turn Right")

    def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
        """Complete the velocity_command arguments

            Use custom mobility params if they are enabled.
        """
        if self._custom_params:
            params = self._mobility_params
        else:
            params = None
        self._start_robot_command(desc,
                                  RobotCommandBuilder.velocity_command(
                                      v_x=v_x, v_y=v_y, v_rot=v_rot, params=params),
                                  end_time_secs=time.time() + VELOCITY_CMD_DURATION)

    def _return_to_origin(self):
        """Not used"""
        if self._custom_params:
            params = self._mobility_params
        else:
            params = None
        self._start_robot_command('fwd_and_rotate',
                                  RobotCommandBuilder.trajectory_command(
                                      goal_x=0.0, goal_y=0.0, goal_heading=0.0,
                                      frame=geometry_pb2.Frame(base_frame=geometry_pb2.FRAME_KO),
                                      params=params, body_height=0.0,
                                      locomotion_hint=spot_command_pb2.HINT_AUTO_TROT),
                                  end_time_secs=time.time() + 20)

    def go_to_waypoint(self, time_secs, desc='go_to_waypoint', goal_x=0.0, goal_y=0.0, goal_heading=0.0):
        """Complete the trajectory_command arguments to move to different waypoints

            Use custom mobility params if they are enabled.
        """
        if self._custom_params:
            params = self._mobility_params
        else:
            params = None
        self._start_robot_command(desc,
                                  RobotCommandBuilder.trajectory_command(
                                      goal_x=goal_x, goal_y=goal_y, goal_heading=goal_heading,
                                      frame=geometry_pb2.Frame(base_frame=geometry_pb2.FRAME_KO),
                                      params=params, body_height=0.0,
                                      locomotion_hint=spot_command_pb2.HINT_AUTO),
                                  end_time_secs=time.time() + time_secs)

    def mobility_param_update(self):
        """Update the mobility params"""

        # body_control: body height and pose
        # locomotion_hint: walking gait
        # stair_hint: Stairs are only supported in trot gaits. Using this hint will override some user defaults in
        # order to optimize stair behavior.
        # allow_degraded_perception: Allow the robot to move with degraded perception when there are perception faults.
        # obstacle_params: Control of obstacle avoidance.
        # swing_height: Swing height setting
        position = geometry_pb2.Vec3(z=self._body_height)
        rotation = geometry.EulerZXY(yaw=self._pose_yaw, roll=self._pose_roll, pitch=self._pose_pitch).to_quaternion()
        pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
        point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
        frame = geometry_pb2.Frame(base_frame=geometry_pb2.FRAME_BODY)
        traj = trajectory_pb2.SE3Trajectory(points=[point], frame=frame)
        body_control = spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)
        self._mobility_params = spot_command_pb2.MobilityParams(body_control=body_control,
                                                                locomotion_hint=self._locomotion_hint,
                                                                stair_hint=self._stair,
                                                                allow_degraded_perception=
                                                                self._allow_degraded_perception,
                                                                obstacle_params=self._obstacle_params,
                                                                swing_height=self._swing_height)
        # self._robot.logger.info("{}".format(self._mobility_params))

    def enable_mobility_params(self, enable):
        """Enable/disable the using of mobility params"""
        self._custom_params = enable
        self.mobility_param_update()
        # return to default speed and params if custom parameters options is not used.
        if not self._custom_params:
            self._base_speed = 0.5
            self._mobility_params = None

    def speed_update(self, speed):
        """Update speed of Spot"""
        if self._custom_params:
            self._robot.logger.info("Speed: {} m/s".format(speed))
            self._base_speed = speed

    def locomotion_hint_update(self, locomotion_hint):
        """Update locomotion gait"""
        if not self._stair:
            self._locomotion_hint = locomotion_hint
            self.mobility_param_update()
        else:
            self._robot.logger.info("Gait is locked to Trot during stair mode")

    def obstacle_params_update(self, obstacle_avoidance_padding=0.5, allow_degraded_perception=False,
                               disable_vision_foot_obstacle_avoidance=False,
                               disable_vision_foot_constraint_avoidance=False,
                               disable_vision_body_obstacle_avoidance=False):
        """Update obstacle avoidance params

        Args:
            obstacle_avoidance_padding: avoidance distance in meters
            allow_degraded_perception: Allow the robot to move with degraded perception when there are perception faults
            disable_vision_foot_obstacle_avoidance: Use vision to make the feet avoid obstacles by swinging higher
            disable_vision_foot_constraint_avoidance: Use vision to make the feet avoid constraints like edges of stairs
            disable_vision_body_obstacle_avoidance: Use vision to make the body avoid obstacles
        """
        self._allow_degraded_perception = allow_degraded_perception
        self._obstacle_params = spot_command_pb2.ObstacleParams(
            obstacle_avoidance_padding=obstacle_avoidance_padding,
            disable_vision_foot_obstacle_avoidance=disable_vision_foot_obstacle_avoidance,
            disable_vision_foot_constraint_avoidance=disable_vision_foot_constraint_avoidance,
            disable_vision_body_obstacle_avoidance=disable_vision_body_obstacle_avoidance)
        self.mobility_param_update()

    def swing_height_update(self, swing_height):
        """Update leg swing height

        Args:
            swing_height: height of leg off the ground during locomotion (Low, medium, high)
        """
        self._swing_height = swing_height
        self.mobility_param_update()

    def stair_update(self, stair):
        """Update stair_hint in mobility params

        Args:
            stair: enable or disable
        """
        self._stair = stair
        if self._stair:
            self._locomotion_hint = spot_command_pb2.HINT_AUTO_TROT
            self._robot.logger.info("Stair is only available in Trot gait")
        self.mobility_param_update()

    def body_pose_update(self, body_height=0.0, yaw=0.0, roll=0.0, pitch=0.0):
        """Update body pose in mobility params

        Args:
            body_height: height relative to default height
            yaw: twist around z-axis
            roll: twist around x-axis
            pitch: twist around y_axis
        """
        self._body_height = body_height
        self._pose_yaw = yaw
        self._pose_roll = roll
        self._pose_pitch = pitch
        self.mobility_param_update()

    def power_state(self):
        """Checking Power State

        Returns:
            robot's motor_power_state
        """
        state = self.robot_state
        if not state:
            return None
        return state.power_state.motor_power_state

    def toggle_power(self):
        """Toggle power state

        Returns:

        """
        power_state = self.power_state()
        if power_state is None:
            self.add_message('Could not toggle power because power state is unknown')
            return

        if power_state == robot_state_proto.PowerState.STATE_OFF:
            self._try_grpc("powering-on", self._request_power_on)
            self._robot.logger.info("Powering On")
        else:
            self._try_grpc("powering-off", self._safe_power_off)
            self._robot.logger.info("Powering Off")

    def _request_power_on(self):
        """Power on

        Returns:

        """
        self._power_client.power_command(power_service_proto.PowerCommandRequest.REQUEST_ON)

    def _safe_power_off(self):
        """Safe power off, the robot will attempt to sit first before turning off power

        Returns:

        """
        self._start_robot_command('safe_power_off', RobotCommandBuilder.safe_power_off_command())
        self._robot.logger.info("Safe Power Off")

    def status_update(self):
        """Send status update signal to GUI"""
        self.stat_update.emit()
