"""
Driver class for TCP sender controller.
"""

import threading
import time
from collections import namedtuple
from typing import Annotated, List, Optional
from annotated_types import Len
import numpy as np
import asyncio
import platform

from pynput.keyboard import Controller, Key, Listener

from robosuite.utils.log_utils import ROBOSUITE_DEFAULT_LOGGER

import robosuite.macros as macros
from robosuite.devices import Device
from robosuite.utils.transform_utils import mat2euler

from robosuite.devices.networking import (
    Session, 
    TCPClient, 
    TCPServer, 
    UDPServer, 
    handler, 
    MessageHandler,
    HelloMessage, 
    PoseMessage, 
    HomePoseMessage,
    BeginEpisodeMessage,
    EndEpisodeMessage,
)

DEFAULT_POSE = ([0,0,0], [0,0,0,1])

class TcpSender(Device, MessageHandler):
    """
    A minimalistic driver class for TcpSender.

    Args:
        env (RobotEnv): The environment which contains the robot(s) to control
                        using this device.
        pos_sensitivity (float): Magnitude of input position command scaling
        rot_sensitivity (float): Magnitude of scale input rotation commands scaling
        tcp_port (int): TCP port to listen to
        udp_port (int): UDP port to listen to
    """

    def __init__(
        self,
        env,
        pos_sensitivity=1.0,
        rot_sensitivity=1.0,
        tcp_port=6080,
        udp_port=6080,
    ):
        Device.__init__(self, env)
        MessageHandler.__init__(self)

        print("Opening TcpSender device")

        self.pos_sensitivity = pos_sensitivity
        self.rot_sensitivity = rot_sensitivity

        self.pose = np.eye(4)

        self._display_controls()

        self.single_click_and_hold = False

        self.sessions = set()
        self.tasks = []

        self._tcp_server = TCPServer(port=tcp_port, message_handler=self)
        self._udp_server = UDPServer(port=udp_port, message_handler=self)
        self._reset_state = 0
        self._enabled = False
        self._receiving = False

        loop = asyncio.new_event_loop()
        self.tasks.append(loop.create_task(self.run()))
        thread = threading.Thread(target=loop.run_forever, daemon=True)
        thread.start()

        print("TcpSender device opened")

        # also add a keyboard for aux controls
        self.listener = Listener(on_press=self.on_press, on_release=self.on_release)

        # start listening
        self.listener.start()

    @staticmethod
    def _display_controls():
        """
        Method to pretty print controls.
        """

        def print_command(char, info):
            char += " " * (30 - len(char))
            print("{}\t{}".format(char, info))

        print("")
        print_command("Control", "Command")
        print_command("'Home Pose' button", "reset simulation")
        print_command("'Gripper Open' button (hold)", "open gripper")
        print_command("'Gripper Close' button (hold)", "close gripper")
        print_command("Move controller laterally", "move arm horizontally in x-y plane")
        print_command("Move controller vertically", "move arm vertically")
        print_command("Twist controller about an axis", "rotate arm about a corresponding axis")
        print("")

    def _reset_internal_state(self):
        """
        Resets internal state of controller, except for the reset signal.
        """
        super()._reset_internal_state()
        self.pose = np.eye(4)
        # Reset control
        self._control = np.zeros(6)
        # Reset grasp
        self.single_click_and_hold = False

    def start_control(self):
        """
        Method that should be called externally before controller can
        start receiving commands.
        """
        self._reset_internal_state()
        self._reset_state = 0
        self._enabled = True

    def get_controller_state(self):
        """
        Grabs the current state of the 3D mouse.

        Returns:
            dict: A dictionary containing dpos, orn, unmodified orn, grasp, and reset
        """
        DPOS_SENSITIVITY = 0.005
        DROT_SENSITIVITY = 0.005
        dpos = self.pose[:3,3] * DPOS_SENSITIVITY * self.pos_sensitivity
        drotation = mat2euler(self.pose) * DROT_SENSITIVITY * self.rot_sensitivity

        return dict(
            dpos=dpos,
            rotation=self.pose[:3,:3],
            raw_drotation=drotation,
            grasp=self.control_gripper,
            reset=self._reset_state,
            base_mode=int(self.base_mode),
        )

    @handler(PoseMessage)
    async def handle_PoseMessage(self, session: Session, msg: PoseMessage, timestamp: float):
        # Reshaping pose list outputs transposed transformation matrix
        pose_mat = np.array(msg.pose).reshape((4,4)).transpose()

        axis_mat = np.eye(4)
        axis_mat[1:3,1:3]=[[0,-1],[1,0]]
        self.pose = np.matmul(axis_mat, pose_mat)

    @handler(HomePoseMessage)
    async def handle_HomePoseMessage(self, session: Session, msg: HomePoseMessage, timestamp: float):
        self._reset_state = 1
        self._enabled = False
        self._reset_internal_state()

    @handler(BeginEpisodeMessage)
    async def handle_BeginEpisodeMessage(self, session: Session, msg: BeginEpisodeMessage, timestamp: float):
        self._receiving = True
    
    @handler(EndEpisodeMessage)
    async def handle_EndEpisodeMessage(self, session: Session, msg: EndEpisodeMessage, timestamp: float):
        self._receiving = False
        self._prev_pose = DEFAULT_POSE
        self._pose = DEFAULT_POSE

    @property
    def control(self):
        """
        Grabs current pose of controller

        Returns:
            np.array: 6-DoF control value
        """
        return np.array(self._control)

    @property
    def control_gripper(self):
        """
        Maps internal states into gripper commands.

        Returns:
            float: Whether we're using single click and hold or not
        """
        if self.single_click_and_hold:
            return 1.0
        return 0

    def on_press(self, key):
        """
        Key handler for key presses.
        Args:
            key (str): key that was pressed
        """
        pass

    def on_release(self, key):
        """
        Key handler for key releases.
        Args:
            key (str): key that was pressed
        """
        try:
            # controls for mobile base (only applicable if mobile base present)
            if key.char == "b":
                self.base_modes[self.active_robot] = not self.base_modes[self.active_robot]  # toggle mobile base
            elif key.char == "s":
                self.active_arm_index = (self.active_arm_index + 1) % len(self.all_robot_arms[self.active_robot])
            elif key.char == "=":
                self.active_robot = (self.active_robot + 1) % self.num_robots

        except AttributeError as e:
            pass

    def _postprocess_device_outputs(self, dpos, drotation):
        drotation = drotation * 1
        dpos = dpos * 1

        dpos = np.clip(dpos, -1, 1)
        drotation = np.clip(drotation, -1, 1)

        return dpos, drotation

    async def run(self):
        await asyncio.gather(
            self._tcp_server.run(),
            self._udp_server.run()
        )

    async def on_connect(self, session: Session):
        print("Connection from: %s" % session.remote_endpoint)
        await session.send(HelloMessage(message = "Hello from robosuite running on %s %s" % (platform.system(), platform.release())))
        self.sessions.add(session)

    async def on_disconnect(self, session: Session):
        print("Disconnected from: %s" % session.remote_endpoint)
        self.sessions.remove(session)

if __name__ == "__main__":

    tcp_sender = TcpSender()
    for i in range(100):
        print(tcp_sender.pose)
        time.sleep(0.02)
