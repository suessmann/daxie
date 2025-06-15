import numpy as np
import pyroki as pk
import torch
import viser
import yourdfpy
from scipy.spatial.transform import Rotation as R
from viser.extras import ViserUrdf

from .solve_ik import solve_ik
from ..server.pos_grpc_server import start_grpc_server

# Constants
INITIAL_WXYZ_ROBOT = np.array([0, 0, 1, 0])
INITIAL_POS_ROBOT = np.array([0.0, -0.17, 0.237])
REST_POSE = np.array([0.0, 3.19, 3.02, 1.0, 0.0, 0.0])
# REST_POSE = np.array(
#     [-0.07409113, -1.73330804, 1.73777044, 1.1190557, -0.0132125, 0.02372983]
# )
SENSITIVITY = {"normal": 0.5, "precision": 0.2}


class VirtualManipulator:
    """Virtual manipulator that maps phone input to robot control."""

    def __init__(self, urdf_path: str, mesh_path: str) -> None:
        self.urdf = yourdfpy.URDF.load(urdf_path, mesh_dir=mesh_path)
        self.robot = pk.Robot.from_urdf(self.urdf)
        self.rotation_sensitivity = 1.0
        self.start_teleop = False
        self._phone_connected = False
        self.target_link_name = "Fixed_Jaw"

        self.current_t_R: np.ndarray = INITIAL_POS_ROBOT
        self.current_q_R: np.ndarray = INITIAL_WXYZ_ROBOT
        self.last_precision_mode = False
        self.dry_run = True

        self._init_visualization()
        self._start_grpc_server()

    def _init_visualization(self) -> None:
        """Initialize the Viser visualization server and URDF model."""
        self.server = viser.ViserServer()
        self.server.scene.add_grid("/ground", width=2.0, height=2.0)
        self.urdf_vis = ViserUrdf(self.server, self.urdf, root_node_name="/base")

    def _start_grpc_server(self) -> None:
        """Start the gRPC server for phone pose streaming."""
        self.grpc_server, self.pose_service = start_grpc_server()
        self.hz_grpc = 0.0
        self.pose_service.get_latest_pose(block=False)

    def _open_phone_connection(
        self, init_qpos: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, bool]:
        """Wait for phone to connect and set initial mapping."""
        init_rot_robot = R.from_quat(self.current_q_R, scalar_first=True)
        self.current_t_R: np.ndarray = INITIAL_POS_ROBOT
        self.current_q_R: np.ndarray = INITIAL_WXYZ_ROBOT

        while not self.start_teleop:
            data = self.pose_service.get_latest_pose(block=True, timeout=100.0)
            self.start_teleop = data["switch"]

        pos, quat, gripper = data["position"], data["rotation"], data["gripper_open"]
        initial_rot_phone = R.from_quat(quat, scalar_first=True)
        initial_pos_phone = np.array(pos)

        self.initial_phone_quat = quat.copy()
        self.initial_phone_pos = initial_pos_phone.copy()

        quat_RP = init_rot_robot * initial_rot_phone.inv()
        translation_RP = self.current_t_R - quat_RP.apply(initial_pos_phone)
        return quat_RP, translation_RP, gripper

    def _reset_mapping(self, phone_pos: np.ndarray, phone_quat: np.ndarray) -> None:
        """Reset mapping parameters when precision mode toggles."""
        self.initial_phone_pos = phone_pos.copy()
        self.initial_phone_quat = phone_quat.copy()

        rot_init = R.from_quat(self.initial_phone_quat, scalar_first=True)
        rot_curr = R.from_quat(self.current_q_R, scalar_first=True)
        self.quat_RP = rot_curr * rot_init.inv()
        self.translation_RP = self.current_t_R - self.quat_RP.apply(
            self.initial_phone_pos
        )

    def _map_phone_to_robot(
        self, phone_pos: np.ndarray, phone_quat: np.ndarray, precision_mode: bool
    ) -> tuple[np.ndarray, np.ndarray]:
        """Map phone translation and rotation to robot's coordinate frame."""
        phone_pos = np.array(phone_pos, float)
        phone_quat = np.array(phone_quat, float)

        if precision_mode != self.last_precision_mode:
            self._reset_mapping(phone_pos, phone_quat)

        self.last_precision_mode = precision_mode
        scale = SENSITIVITY["precision"] if precision_mode else SENSITIVITY["normal"]

        # Translate
        delta = (phone_pos - self.initial_phone_pos) * scale
        scaled_pos = self.initial_phone_pos + delta

        # Rotate
        init_rot = R.from_quat(self.initial_phone_quat, scalar_first=True)
        curr_rot = R.from_quat(phone_quat, scalar_first=True)
        relative_rot = init_rot.inv() * curr_rot
        rotvec = relative_rot.as_rotvec() * self.rotation_sensitivity
        scaled_rot = R.from_rotvec(rotvec)
        quat_scaled = init_rot * scaled_rot

        # Apply mapping
        quat_robot = self.quat_RP * quat_scaled
        pos_robot = self.quat_RP.apply(scaled_pos) + self.translation_RP

        self.current_q_R = quat_robot.as_quat(scalar_first=True)
        self.current_t_R = pos_robot
        return pos_robot, self.current_q_R

    def get_rest_pose(self):
        return REST_POSE

    def render_step(self, joints_deg: list[float]) -> None:
        """Update visualization with current joint angles (degrees)."""
        # joints_rad = np.deg2rad(joints_deg)
        self.urdf_vis.update_cfg(joints_deg)

    def send_to_robot(self, curr_qpos: list[float]) -> torch.Tensor:
        """Process phone input, solve IK, and send commands to the robot."""
        curr_qpos_rad = np.deg2rad(curr_qpos)

        if not self._phone_connected:
            self.quat_RP, self.translation_RP, _ = self._open_phone_connection(
                curr_qpos_rad
            )
            self._phone_connected = True

        if not self.start_teleop:
            self._phone_connected = False
            return torch.tensor(np.rad2deg(REST_POSE))

        data = self.pose_service.get_latest_pose(block=False)
        pos, quat, gripper = data["position"], data["rotation"], data["gripper_open"]

        # Map phone pose to robot pose
        t_robot, q_robot = self._map_phone_to_robot(pos, quat, data["precision"])

        # Solve inverse kinematics
        solution = solve_ik(
            robot=self.robot,
            target_link_name=self.target_link_name,
            target_position=t_robot,
            target_wxyz=q_robot,
        )

        self.render_step(solution)

        # Update gripper state
        solution[-1] = 0.875 if gripper else 0.0
        self.urdf_vis.update_cfg(solution)
        self.start_teleop = data["switch"]

        # if self.dry_run:
        #     return torch.tensor(np.rad2deg(curr_qpos_rad))

        return torch.tensor(np.rad2deg(solution))
