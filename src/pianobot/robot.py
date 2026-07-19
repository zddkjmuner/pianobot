from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pydrake.geometry as geometry
from manipulation.scenarios import AddShape
from pydrake.all import (
    DiagramBuilder,
    FixedOffsetFrame,
    InverseKinematics,
    MeshcatVisualizer,
    MultibodyPlant,
    MultibodyPositionToGeometryPose,
    PackageMap,
    Parser,
    PiecewisePolynomial,
    PiecewiseQuaternionSlerp,
    Quaternion,
    RevoluteJoint,
    RigidTransform,
    RotationMatrix,
    SceneGraph,
    Simulator,
    Solve,
    StartMeshcat,
    TrajectorySource,
)
from pydrake.multibody.tree import DoorHinge, DoorHingeConfig

from .scores import MusicNote, generate_note_from_chord


@dataclass(slots=True)
class KeyboardKey:
    name: str
    pos: np.ndarray
    rotation: RotationMatrix
    frame: object
    keytype: str


def _resolve_legacy_resource(resource_path: str) -> str:
    legacy_prefix = "drake/manipulation/models/"
    if resource_path.startswith(legacy_prefix):
        relative_path = resource_path.removeprefix(legacy_prefix)
        relative_path = relative_path.replace("iiwa_description/iiwa7/", "iiwa_description/sdf/")
        return PackageMap().ResolveUrl(f"package://drake_models/{relative_path}")
    from pydrake.all import FindResourceOrThrow

    return FindResourceOrThrow(resource_path)


def _add_model_from_file(plant, scene_graph, file_name: str) -> int:
    parser = Parser(plant, scene_graph)
    return parser.AddModels(file_name)[0]


class PianoRobotPlanner:
    def __init__(self, time_step: float = 1e-5):
        self.builder = DiagramBuilder()
        self.plant = MultibodyPlant(time_step=time_step)
        self.scene_graph = self.builder.AddSystem(SceneGraph())
        self.plant.RegisterAsSourceForSceneGraph(self.scene_graph)
        self.time_step = time_step
        self.mu = 1.0

        self._build_plant()
        self._initialize_plant_metadata()

    def _build_plant(self) -> None:
        color_black = [0, 0, 0, 1]
        color_white = [1, 1, 1, 1]
        color_base = [0.3, 0.2, 0.1, 1]

        # Build ground as in notebook via AddShape helper.
        AddShape(self.plant, geometry.Box(10, 10, 2.0), name="ground", mu=self.mu)
        self.plant.WeldFrames(
            self.plant.world_frame(), self.plant.GetFrameByName("ground"), RigidTransform(p=[0, 0, -1.0])
        )
        self.plant.AddFrame(
            FixedOffsetFrame("frame_top_ground", self.plant.GetFrameByName("ground"), RigidTransform(p=[0, 0, 1]))
        )

        self.base_name = "piano_base"
        self.base_size = [1.8, 0.5, 0.2]
        self.base_pos = [-0.20, 0.9, 0]

        key_scale = 2.2
        key_size = np.array([0.019, 0.04, 0.02]) * key_scale
        key_offset = np.array([0.0042, 0.0, 0]) * key_scale
        key_minor_size = key_size.copy()
        key_minor_size[0] *= 0.75
        self.key_size = key_size

        AddShape(
            self.plant,
                geometry.Box(self.base_size[0], self.base_size[1], self.base_size[2]),
            name=self.base_name,
            mass=10,
            mu=self.mu,
            color=color_base,
        )
        self.plant.AddFrame(
            FixedOffsetFrame(
                "frame_bottom_" + self.base_name,
                self.plant.GetFrameByName(self.base_name),
                RigidTransform(p=[0, 0, -self.base_size[2] * 0.5]),
            )
        )
        self.plant.AddFrame(
            FixedOffsetFrame(
                "frame_top_" + self.base_name,
                self.plant.GetFrameByName(self.base_name),
                RigidTransform(p=[0, 0, self.base_size[2] * 0.5]),
            )
        )
        self.plant.WeldFrames(
            self.plant.GetFrameByName("frame_top_ground"),
            self.plant.GetFrameByName("frame_bottom_" + self.base_name),
            RigidTransform(p=self.base_pos),
        )

        def add_key_hinge(name: str, key_pos: np.ndarray, size: np.ndarray, color=color_white) -> None:
            key_base_name = "key_anchor_" + name
            key_name = "key_" + name
            key_moment_arm = size[1] * 7
            color_key_base = [0.4, 0.2, 0.1, 1]

            AddShape(
                self.plant,
                geometry.Box(size[0], size[0] * 0.5, size[2] * 0.1),
                name=key_base_name,
                mass=1,
                mu=1,
                color=color_key_base,
            )
            self.plant.AddFrame(
                FixedOffsetFrame(
                    "frame_bottom_" + key_base_name,
                    self.plant.GetFrameByName(key_base_name),
                    RigidTransform(p=[0, 0, -size[2] * 0.05]),
                )
            )
            self.plant.AddFrame(
                FixedOffsetFrame(
                    "frame_top_" + key_base_name,
                    self.plant.GetFrameByName(key_base_name),
                    RigidTransform(p=[0, 0, +size[2] * 0.05 + 0.05]),
                )
            )

            hinge_offset = np.array([0, key_moment_arm, size[2] * 2])
            self.plant.WeldFrames(
                self.plant.GetFrameByName("frame_top_" + self.base_name),
                self.plant.GetFrameByName("frame_bottom_" + key_base_name),
                RigidTransform(p=hinge_offset + key_pos),
            )

            AddShape(
                self.plant,
                geometry.Box(size[0], size[1], size[2]),
                name=key_name,
                mass=0.1,
                mu=self.mu,
                color=color,
            )
            self.plant.AddFrame(
                FixedOffsetFrame(
                    "frame_rev_" + key_name,
                    self.plant.GetFrameByName(key_name),
                    RigidTransform(p=[0, key_moment_arm, 0]),
                )
            )

            x_kt = RigidTransform(p=[0, 0, size[2] * 0.25])
            x_kr1 = RigidTransform(R=RotationMatrix.MakeYRotation(np.pi))
            x_kr2 = RigidTransform(R=RotationMatrix.MakeZRotation(-np.pi / 2))
            self.plant.AddFrame(
                FixedOffsetFrame(
                    "frame_keycontact_" + key_name,
                    self.plant.GetFrameByName(key_name),
                    x_kt.multiply(x_kr1.multiply(x_kr2)),
                )
            )

            key_joint = self.plant.AddJoint(
                RevoluteJoint(
                    name="revolve_joint_" + key_name,
                    frame_on_parent=self.plant.GetFrameByName(key_base_name),
                    frame_on_child=self.plant.GetFrameByName("frame_rev_" + key_name),
                    axis=[1, 0, 0],
                    damping=0.0,
                )
            )

            cfg = DoorHingeConfig()
            cfg.spring_constant = 5
            cfg.spring_zero_angle_rad = 1
            cfg.catch_torque = cfg.spring_constant * 1.55
            cfg.catch_width = cfg.spring_zero_angle_rad
            cfg.viscous_friction = cfg.spring_constant * 0.2
            cfg.static_friction_torque = cfg.spring_constant * 0.01
            cfg.dynamic_friction_torque = cfg.spring_constant * 0.0
            self.plant.AddForceElement(DoorHinge(joint=key_joint, config=cfg))

        key_major_list = ["C", "D", "E", "F", "G", "A", "B"]
        key_minor_list = ["C#", "D#", "E#", "F#", "G#", "A#", "B#"]
        key_major_pos = np.array([-(key_size[0] + key_offset[0]) * (7 * 3 / 2) * 2 + 0.2, -self.base_size[1] * 0.5, 0])

        self.key_list: list[str] = []
        for s in range(5):
            for i in range(7):
                key_name = key_major_list[i] + f"_{s}"
                add_key_hinge(key_name, key_major_pos, key_size)
                self.key_list.append(key_name)

                if i in {0, 1, 3, 4, 5}:
                    key_minor_name = key_minor_list[i] + f"_{s}"
                    key_minor_pos = key_major_pos.copy()
                    key_minor_pos[0] += (key_size[0] + key_offset[0]) * 0.5
                    key_minor_pos[1] += key_size[1] + 0.005
                    key_minor_pos[2] += key_size[2] * 0.5
                    add_key_hinge(key_minor_name, key_minor_pos, key_minor_size, color=color_black)
                    self.key_list.append(key_minor_name)

                key_major_pos[0] += key_size[0] + key_offset[0]

        robot_file = _resolve_legacy_resource(
            "drake/manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf"
        )
        hand_file = _resolve_legacy_resource(
            "drake/manipulation/models/allegro_hand_description/sdf/allegro_hand_description_right.sdf"
        )

        self.model_arm = _add_model_from_file(self.plant, self.scene_graph, robot_file)
        self.plant.WeldFrames(
            self.plant.GetFrameByName("frame_top_ground"),
            self.plant.get_body(self.plant.GetBodyIndices(self.model_arm)[0]).body_frame(),
            RigidTransform(p=[0, 0, 0.0001]),
        )
        self.model_hand = _add_model_from_file(self.plant, self.scene_graph, hand_file)
        self.plant.AddFrame(
            FixedOffsetFrame("frame_palm", self.plant.GetFrameByName("hand_root"), RigidTransform(p=[0, 0, 0.05]))
        )
        self.plant.WeldFrames(
            self.plant.get_body(self.plant.GetBodyIndices(self.model_arm)[7]).body_frame(),
            self.plant.get_body(self.plant.GetBodyIndices(self.model_hand)[0]).body_frame(),
            RigidTransform(p=[0, 0, 0.05]),
        )

        self.finger_link_list = ["link_15", "link_3", "link_7", "link_11"]
        self.finger_connect_list = ["link_14", "link_2", "link_6", "link_10"]
        for body_name in self.finger_link_list:
            self.plant.AddFrame(
                FixedOffsetFrame(
                    "frame_fingercontact_" + body_name,
                    self.plant.GetFrameByName(body_name),
                    RigidTransform(p=[0, 0, 0.032]),
                )
            )

        self.plant.mutable_gravity_field().set_gravity_vector([0, 0, 0])
        self.plant.Finalize()
        self.plant_context = self.plant.CreateDefaultContext()

    def _initialize_plant_metadata(self) -> None:
        self.fingers = [self.plant.GetBodyByName(name) for name in self.finger_link_list]
        self.finger_frames = [self.plant.GetFrameByName("frame_fingercontact_" + name) for name in self.finger_link_list]
        self.finger_link_frames = [self.plant.GetFrameByName(name) for name in self.finger_connect_list]
        self.palm_frame = self.plant.GetFrameByName("frame_palm")

        self.note_dictionary: dict[str, KeyboardKey] = {}
        for idx, key in enumerate(self.key_list):
            key_pose = self.plant.CalcRelativeTransform(
                self.plant_context,
                frame_A=self.plant.GetFrameByName("frame_top_ground"),
                frame_B=self.plant.GetFrameByName("frame_keycontact_key_" + key),
            )
            k_type = idx % 12
            key_type = "sharp" if k_type in {1, 3, 6, 8, 10} else "flat"
            self.note_dictionary[key] = KeyboardKey(
                key,
                pos=key_pose.translation(),
                rotation=key_pose.rotation(),
                frame=self.plant.GetFrameByName("frame_keycontact_key_" + key),
                keytype=key_type,
            )
        self.music_note_list = list(self.note_dictionary.keys())

    def get_finger_pose(self, index: int) -> RigidTransform:
        return self.plant.EvalBodyPoseInWorld(self.plant_context, self.fingers[index])

    def find_finger_correspondence(self, key_list: list[str]) -> tuple[int, np.ndarray, np.ndarray]:
        n_finger = 4
        n_key = len(key_list)
        if n_key >= 4:
            return 1, np.array([[0, 1, 2, 3]], dtype=float), np.array([[1.0]])
        if n_key <= 0:
            return 0, np.array([[-1.0]]), np.array([[-1.0]])

        dist = np.zeros((n_key, n_finger))
        for key_idx, key in enumerate(key_list):
            for f in range(n_finger):
                note = self.note_dictionary[key]
                p_fk = note.pos - self.get_finger_pose(f).translation()
                dist[key_idx, f] = np.dot(p_fk[:2], p_fk[:2])

        if n_key == 1:
            fidx = np.array([[1], [2], [3]], dtype=float)
            cost = np.array([[dist[0, 1]], [dist[0, 2]], [dist[0, 3]]])
            return 3, fidx, cost
        if n_key == 2:
            combos = []
            costs = []
            for f1 in range(n_finger):
                for f2 in range(f1 + 1, n_finger):
                    combos.append([f1, f2])
                    costs.append([dist[0, f1] + dist[0, f2]])
            return 6, np.array(combos, dtype=float), np.array(costs)

        combos = []
        costs = []
        for f1 in range(n_finger):
            for f2 in range(f1 + 1, n_finger):
                for f3 in range(f2 + 1, n_finger):
                    combos.append([f1, f2, f3])
                    costs.append([dist[0, f1] + dist[0, f2] + dist[0, f3]])
        return 4, np.array(combos, dtype=float), np.array(costs)

    def ik_init(self, q0: np.ndarray) -> tuple[bool, np.ndarray, np.ndarray, np.ndarray]:
        ik = InverseKinematics(self.plant, self.plant_context)
        t_center = self.note_dictionary[self.key_list[18]].pos
        t_palm_lower = t_center + np.array([-0.01, -self.key_size[1] * 4 - 0.01, self.key_size[2] * 3])
        t_palm_upper = t_center + np.array([0.01, -self.key_size[1] * 2 + 0.01, self.key_size[2] * 5 + 0.01])

        ik.AddPositionConstraint(self.palm_frame, [0, 0, 0], self.plant.world_frame(), t_palm_lower, t_palm_upper)
        ik.AddAngleBetweenVectorsConstraint(self.palm_frame, [0, 0, 1], self.plant.world_frame(), [0, 1, 0], 0, np.pi / 2 * 0.05)
        ik.AddAngleBetweenVectorsConstraint(self.palm_frame, [1, 0, 0], self.plant.world_frame(), [0, 0, -1], 0, np.pi / 2 * 0.05)
        ik.AddAngleBetweenVectorsConstraint(self.palm_frame, [0, 1, 0], self.plant.world_frame(), [-1, 0, 0], 0, np.pi / 2 * 0.05)

        prog = ik.get_mutable_prog()
        q = ik.q()
        prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
        prog.SetInitialGuess(q, q0)
        result = Solve(ik.prog())

        q_iiwa = self.plant.GetPositions(self.plant_context, self.plant.GetModelInstanceByName("iiwa7"))
        q_hand = self.plant.GetPositions(self.plant_context, self.plant.GetModelInstanceByName("allegro_hand_right"))
        return result.is_success(), q_iiwa, q_hand, result.GetSolution()

    def ik_prepress_chord(self, q0: np.ndarray, note_on_fingers: list[str]) -> tuple[bool, np.ndarray, np.ndarray]:
        ik = InverseKinematics(self.plant, self.plant_context)

        t_finger = []
        t_link = []
        for i in range(4):
            t_finger.append(self.plant.CalcRelativeTransform(self.plant_context, self.plant.GetFrameByName("frame_top_ground"), self.finger_frames[i]).translation())
            t_link.append(self.plant.CalcRelativeTransform(self.plant_context, self.plant.GetFrameByName("frame_top_ground"), self.finger_link_frames[i]).translation())

        t_palm = self.plant.CalcRelativeTransform(self.plant_context, self.plant.GetFrameByName("frame_top_ground"), self.palm_frame).translation().copy()
        tip_offset = np.array([0, 0, self.key_size[2] * 1.6])

        flat_lower = np.array([-self.key_size[0] * 0.15, -self.key_size[1] * 0.4, -self.key_size[2] * 0.01])
        flat_upper = np.array([self.key_size[0] * 0.15, self.key_size[1] * 0.15, self.key_size[2]])
        sharp_lower = np.array([-self.key_size[0] * 0.15, -self.key_size[1] * 0.3, -self.key_size[2] * 0.01])
        sharp_upper = np.array([self.key_size[0] * 0.15, self.key_size[1] * 0.2, self.key_size[2]])
        none_lower = np.array([-self.key_size[0], -self.key_size[1] * 0.5, self.key_size[2] * 0.01])
        none_upper = np.array([self.key_size[0], self.key_size[1] * 0.5, self.key_size[2] * 1.5])

        for idx, note in enumerate(note_on_fingers):
            t_key = t_finger[idx].copy()
            t_keylink = t_link[idx].copy()
            if note is not None and "none" in note:
                t_lower = t_key + none_lower
                t_upper = t_key + none_upper
                t_link_lower = t_keylink + none_lower
                t_link_upper = t_keylink + none_upper
            else:
                key_type = self.note_dictionary[note].keytype
                t_key += tip_offset
                t_keylink += tip_offset
                if key_type == "flat":
                    t_lower, t_upper = t_key + flat_lower, t_key + flat_upper
                else:
                    t_lower, t_upper = t_key + sharp_lower, t_key + sharp_upper
                t_link_lower, t_link_upper = t_keylink + none_lower, t_keylink + none_upper

            ik.AddPositionConstraint(self.finger_frames[idx], [0, 0, 0], self.plant.world_frame(), t_lower, t_upper)
            ik.AddPositionConstraint(self.finger_link_frames[idx], [0, 0, 0], self.plant.world_frame(), t_link_lower, t_link_upper)

        t_palm += tip_offset
        palm_lower = t_palm + np.array([-self.key_size[0] * 2, -self.key_size[1] * 0.4, -self.key_size[2] * 0.3])
        palm_upper = t_palm + np.array([self.key_size[0] * 2, self.key_size[1] * 0.45, self.key_size[2] * 2.5])
        ik.AddPositionConstraint(self.palm_frame, [0, 0, 0], self.plant.world_frame(), palm_lower, palm_upper)
        ik.AddAngleBetweenVectorsConstraint(self.palm_frame, [0, 0, 1], self.plant.world_frame(), [0, 1, 0], 0, np.pi / 2 * 0.67)

        prog = ik.get_mutable_prog()
        q = ik.q()
        prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
        prog.SetInitialGuess(q, q0)
        result = Solve(ik.prog())
        q_iiwa = self.plant.GetPositions(self.plant_context, self.plant.GetModelInstanceByName("iiwa7"))
        q_hand = self.plant.GetPositions(self.plant_context, self.plant.GetModelInstanceByName("allegro_hand_right"))
        return result.is_success(), q_iiwa, q_hand

    def ik_press_chord(self, q0: np.ndarray, note_on_fingers: list[str]) -> tuple[bool, np.ndarray, np.ndarray]:
        ik = InverseKinematics(self.plant, self.plant_context)

        flat_lower = np.array([-self.key_size[0] * 0.005, -self.key_size[1] * 0.3, -self.key_size[2] * 0.05])
        flat_upper = np.array([self.key_size[0] * 0.005, self.key_size[1] * 0.1, self.key_size[2] * 0.2])
        sharp_lower = np.array([-self.key_size[0] * 0.005, -self.key_size[1] * 0.35, -self.key_size[2] * 0.05])
        sharp_upper = np.array([self.key_size[0] * 0.005, self.key_size[1] * 0.3, self.key_size[2] * 0.2])

        t_center = self.note_dictionary[self.key_list[18]].pos
        release_lower = np.array([-1, -1, self.key_size[2] * 1.2])
        release_upper = np.array([1, 1, self.key_size[2] * 10])

        t_average = np.array([0.0, 0.0, 0.0])
        n_keys = 0
        for idx, note in enumerate(note_on_fingers):
            if note is not None and "none" in note:
                t_key = t_center
                t_lower, t_upper = t_key + release_lower, t_key + release_upper
            else:
                key_type = self.note_dictionary[note].keytype
                t_key = self.note_dictionary[note].pos
                if key_type == "flat":
                    t_lower, t_upper = t_key + flat_lower, t_key + flat_upper
                else:
                    t_lower, t_upper = t_key + sharp_lower, t_key + sharp_upper
                t_average += t_key
                n_keys += 1

            ik.AddPositionConstraint(self.finger_frames[idx], [0, 0, 0], self.plant.world_frame(), t_lower, t_upper)

        t_average = t_average / n_keys if n_keys > 0 else t_center
        if n_keys > 0:
            palm_lower = t_average + np.array([-1, -self.key_size[1] * 5, self.key_size[2] * 1])
            palm_upper = t_average + np.array([1, self.key_size[1] * 5, self.key_size[2] * 10])
            ik.AddAngleBetweenVectorsConstraint(self.palm_frame, [0, 0, 1], self.plant.world_frame(), [0, 1, 0], 0, np.pi / 2 * (0.6 if n_keys == 1 else 0.8))
            ik.AddAngleBetweenVectorsConstraint(self.palm_frame, [1, 0, 0], self.plant.world_frame(), [0, 0, -1], 0, np.pi / 2 * 0.5)
        else:
            palm_lower = t_average + np.array([-1, -self.key_size[1] * 5, self.key_size[2] * 1.5])
            palm_upper = t_average + np.array([1, 0.5, self.key_size[2] * 10])

        ik.AddPositionConstraint(self.palm_frame, [0, 0, 0], self.plant.world_frame(), palm_lower, palm_upper)

        prog = ik.get_mutable_prog()
        q = ik.q()
        prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
        prog.SetInitialGuess(q, q0)
        result = Solve(ik.prog())

        q_iiwa = self.plant.GetPositions(self.plant_context, self.plant.GetModelInstanceByName("iiwa7"))
        q_hand = self.plant.GetPositions(self.plant_context, self.plant.GetModelInstanceByName("allegro_hand_right"))
        return result.is_success(), q_iiwa, q_hand

    def ik_transition_arm(
        self, q_start: np.ndarray, q_end: np.ndarray, n_list: int = 1
    ) -> tuple[list[float], list[np.ndarray], list[np.ndarray], list[np.ndarray]]:
        ik = InverseKinematics(self.plant, self.plant_context)

        def get_pose(frame):
            return self.plant.CalcRelativeTransform(
                self.plant_context,
                frame_A=self.plant.GetFrameByName("frame_top_ground"),
                frame_B=frame,
            )

        self.plant.SetPositions(self.plant_context, q_start)
        x_start = get_pose(self.palm_frame)
        self.plant.SetPositions(self.plant_context, q_end)
        x_end = get_pose(self.palm_frame)

        palm_t = PiecewisePolynomial.FirstOrderHold(
            [0.0, 1.0], np.vstack([[x_start.translation()], [x_end.translation()]]).T
        )
        palm_r = PiecewiseQuaternionSlerp()
        palm_r.Append(0.0, x_start.rotation())
        palm_r.Append(1.0, x_end.rotation())

        q0s = q_start
        q0e = q_end
        t_sol: list[float] = []
        q_iiwa_sol: list[np.ndarray] = []
        q_hand_sol: list[np.ndarray] = []
        q_sol: list[np.ndarray] = []

        n = n_list
        for i in range(n_list):
            t = float(i + 1) / (n_list + 1)
            if i % 2 == 0:
                t_now = 1 / (n + 1)
                r_now = palm_r.value(t_now)
                q0 = q0s * (1 - t_now) + q0e * t_now
            else:
                t_now = float(n / (n + 1))
                r_now = palm_r.value(t_now)
                q0 = q0s * (1 - t_now) + q0e * t_now

            t_now_val = palm_t.value(t_now)
            t_offset = np.array([[0.001], [0.001], [0.001]])
            ik.AddPositionConstraint(self.palm_frame, [0, 0, 0], self.plant.world_frame(), t_now_val - t_offset, t_now_val + t_offset)
            ik.AddOrientationConstraint(
                frameAbar=self.palm_frame,
                R_AbarA=RotationMatrix(),
                frameBbar=self.plant.world_frame(),
                R_BbarB=RotationMatrix(Quaternion(np.asarray(r_now).reshape(-1))),
                theta_bound=0.8,
            )

            prog = ik.get_mutable_prog()
            q = ik.q()
            prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
            prog.SetInitialGuess(q, q0)
            Solve(ik.prog())

            q_now = self.plant.GetPositions(self.plant_context)
            q_iiwa = self.plant.GetPositions(self.plant_context, self.plant.GetModelInstanceByName("iiwa7"))
            q_hand = self.plant.GetPositions(self.plant_context, self.plant.GetModelInstanceByName("allegro_hand_right"))
            if i % 2 == 0:
                q0s = q_now
            else:
                q0e = q_now
            t_sol.append(t)
            q_iiwa_sol.append(q_iiwa)
            q_hand_sol.append(q_hand)
            q_sol.append(q_now)
            n -= 1

        return t_sol, q_iiwa_sol, q_hand_sol, q_sol

    def ik_chord_to_finger(
        self, chord: str, octave: int, q0: np.ndarray
    ) -> tuple[bool, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, list[str]]:
        notes_key = generate_note_from_chord(chord, octave)
        n_sol, fidx, cost = self.find_finger_correspondence(notes_key)
        cost_idx = np.argsort(cost[:, 0])

        self.plant.SetPositions(self.plant_context, q0)
        result = False
        q_iiwa_prepress = q_hand_prepress = q_iiwa_press = q_hand_press = q_iiwa_release = q_hand_release = None
        notes = ["none", "none", "none", "none"]
        for i in range(n_sol):
            notes = ["none", "none", "none", "none"]
            n_idx = 0
            for ff in fidx[cost_idx[i]]:
                notes[int(ff)] = notes_key[n_idx]
                n_idx += 1

            result_prepress, q_iiwa_prepress, q_hand_prepress = self.ik_prepress_chord(q0, notes)
            q0_prepress = self.plant.GetPositions(self.plant_context)
            result_press, q_iiwa_press, q_hand_press = self.ik_press_chord(q0_prepress, notes)
            q0_press = self.plant.GetPositions(self.plant_context)
            result_release, q_iiwa_release, q_hand_release = self.ik_prepress_chord(q0_press, notes)
            result = result_prepress and result_press and result_release
            if result:
                break

        if not result:
            notes = ["none", "none", "none", "none"]
            n_idx = 0
            for ff in fidx[cost_idx[0]]:
                notes[int(ff)] = notes_key[n_idx]
                n_idx += 1
            _, q_iiwa_prepress, q_hand_prepress = self.ik_prepress_chord(q0, notes)
            q0_prepress = self.plant.GetPositions(self.plant_context)
            _, q_iiwa_press, q_hand_press = self.ik_press_chord(q0_prepress, notes)
            q0_press = self.plant.GetPositions(self.plant_context)
            _, q_iiwa_release, q_hand_release = self.ik_prepress_chord(q0_press, notes)

        return (
            bool(result),
            q_iiwa_prepress,
            q_hand_prepress,
            q_iiwa_press,
            q_hand_press,
            q_iiwa_release,
            q_hand_release,
            notes,
        )

    def music_sequence_to_trajectory(
        self, music_seq: list[MusicNote]
    ) -> tuple[
        PiecewisePolynomial,
        PiecewisePolynomial,
        PiecewisePolynomial,
        PiecewisePolynomial,
        PiecewisePolynomial,
        PiecewisePolynomial,
        list[list[object]],
        np.ndarray,
    ]:
        iiwa = self.plant.GetModelInstanceByName("iiwa7")
        hand = self.plant.GetModelInstanceByName("allegro_hand_right")

        q0 = self.plant.GetPositions(self.plant_context)
        result_test, q_iiwa_init, q_hand_init, _ = self.ik_init(q0)
        if not result_test:
            raise RuntimeError("ik_init failed")
        q0 = self.plant.GetPositions(self.plant_context)
        q0_init = q0
        t = 0.1

        c_hand_release = np.array([0.001, 0.001, 0.001, 0.001])

        chord_result: list[list[object]] = []

        def notes_to_c_hand(notes: list[str]) -> np.ndarray:
            c_hand = np.array([0.0, 0.0, 0.0, 0.0])
            for idx, note in enumerate(notes):
                if note != "none":
                    c_hand[idx] = 1.0
            return c_hand

        q0_past = self.plant.GetPositions(self.plant_context)
        t_transition_past = 3.0
        init_flag = False

        for music_note in music_seq:
            if music_note.name == "none":
                t_all = music_note.t_push + music_note.t_release + music_note.t_hold + t_transition_past
                t_transition_past = t_all + music_note.t_transition
                continue

            (
                result_press,
                q_iiwa_prepress,
                q_hand_prepress,
                q_iiwa_press,
                q_hand_press,
                q_iiwa_release,
                q_hand_release,
                notes,
            ) = self.ik_chord_to_finger(music_note.name, music_note.key_scale, q0)
            c_hand_push = notes_to_c_hand(notes)

            self.plant.SetPositions(self.plant_context, iiwa, q_iiwa_press)
            self.plant.SetPositions(self.plant_context, hand, q_hand_press)
            q0_press = self.plant.GetPositions(self.plant_context)

            self.plant.SetPositions(self.plant_context, iiwa, q_iiwa_prepress)
            self.plant.SetPositions(self.plant_context, hand, q_hand_prepress)
            q0_prepress = self.plant.GetPositions(self.plant_context)

            self.plant.SetPositions(self.plant_context, iiwa, q_iiwa_release)
            self.plant.SetPositions(self.plant_context, hand, q_hand_release)
            q0_release = self.plant.GetPositions(self.plant_context)

            chord_result.append([music_note.name, str(music_note.key_scale), result_press])

            if not init_flag:
                q_iiwa_traj = PiecewisePolynomial.FirstOrderHold([0, 0.5], np.vstack([q_iiwa_release, q_iiwa_release]).T)
                q_hand_traj = PiecewisePolynomial.FirstOrderHold([0, 0.5], np.vstack([q_hand_release, q_hand_release]).T)
                q_all_traj = PiecewisePolynomial.FirstOrderHold([0, 0.5], np.vstack([q0_release, q0_release]).T)
                c_hand_traj = PiecewisePolynomial.FirstOrderHold([0, 0.5], np.vstack([c_hand_release, c_hand_release]).T)
                self.q0_init = q0_release

                t += t_transition_past
                q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_release)
                q_hand_traj.AppendFirstOrderSegment(t, q_hand_release)
                q_all_traj.AppendFirstOrderSegment(t, q0_release)
                c_hand_traj.AppendFirstOrderSegment(t, c_hand_release)
                init_flag = True
            else:
                t_transition_list, q_arm_list, a_hand_list, q_sol_list = self.ik_transition_arm(q0_past, q0_prepress, 2)
                for idx, t_ratio in enumerate(t_transition_list):
                    t_now = t_transition_past * 0.8 * t_ratio
                    q_iiwa_traj.AppendFirstOrderSegment(t + t_now, q_arm_list[idx])
                    q_hand_traj.AppendFirstOrderSegment(t + t_now, a_hand_list[idx])
                    q_all_traj.AppendFirstOrderSegment(t + t_now, q_sol_list[idx])

                t += t_transition_past * 0.8
                q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_prepress)
                q_hand_traj.AppendFirstOrderSegment(t, q_hand_prepress)
                q_all_traj.AppendFirstOrderSegment(t, q0_prepress)
                c_hand_traj.AppendFirstOrderSegment(t, c_hand_release)

                t += t_transition_past * 0.2
                q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_prepress)
                q_hand_traj.AppendFirstOrderSegment(t, q_hand_prepress)
                q_all_traj.AppendFirstOrderSegment(t, q0_prepress)
                c_hand_traj.AppendFirstOrderSegment(t, c_hand_release)

            t += music_note.t_push
            q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_press)
            q_hand_traj.AppendFirstOrderSegment(t, q_hand_press)
            q_all_traj.AppendFirstOrderSegment(t, q0_press)
            c_hand_traj.AppendFirstOrderSegment(t, c_hand_push)

            t += music_note.t_hold
            q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_press)
            q_hand_traj.AppendFirstOrderSegment(t, q_hand_press)
            q_all_traj.AppendFirstOrderSegment(t, q0_press)
            c_hand_traj.AppendFirstOrderSegment(t, c_hand_push)

            t += music_note.t_release
            q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_release)
            q_hand_traj.AppendFirstOrderSegment(t, q_hand_release)
            q_all_traj.AppendFirstOrderSegment(t, q0_release)
            c_hand_traj.AppendFirstOrderSegment(t, c_hand_release)

            t_transition_past = music_note.t_transition
            q0_past = q0_release
            q0 = q0_release

        t += 1
        q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_release)
        q_hand_traj.AppendFirstOrderSegment(t, q_hand_release)
        q_all_traj.AppendFirstOrderSegment(t, q0_release)
        c_hand_traj.AppendFirstOrderSegment(t, c_hand_release)

        t += 4
        q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_init)
        q_hand_traj.AppendFirstOrderSegment(t, q_hand_init)
        q_all_traj.AppendFirstOrderSegment(t, q0_init)
        c_hand_traj.AppendFirstOrderSegment(t, c_hand_release)

        t += 1
        q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_init)
        q_hand_traj.AppendFirstOrderSegment(t, q_hand_init)
        q_all_traj.AppendFirstOrderSegment(t, q0_init)
        c_hand_traj.AppendFirstOrderSegment(t, c_hand_release)

        w_iiwa_traj = q_iiwa_traj.MakeDerivative()
        w_hand_traj = q_hand_traj.MakeDerivative()

        return q_iiwa_traj, q_hand_traj, q_all_traj, w_iiwa_traj, w_hand_traj, c_hand_traj, chord_result, self.q0_init

    def record_static_html(
        self,
        position_trajectory: PiecewisePolynomial,
        output_html: Path,
        *,
        duration: float | None = None,
    ) -> Path:
        meshcat = StartMeshcat()

        q_source = self.builder.AddSystem(TrajectorySource(position_trajectory))
        q_source.set_name("q_all_source")
        pose_system = self.builder.AddSystem(MultibodyPositionToGeometryPose(self.plant))
        self.builder.Connect(q_source.get_output_port(), pose_system.get_input_port())
        self.builder.Connect(
            pose_system.get_output_port(),
            self.scene_graph.get_source_pose_port(self.plant.get_source_id()),
        )
        visualizer = MeshcatVisualizer.AddToBuilder(self.builder, self.scene_graph, meshcat)

        diagram = self.builder.Build()
        context = diagram.CreateDefaultContext()

        visualizer.StartRecording()
        simulator = Simulator(diagram, context)
        simulator.set_target_realtime_rate(0.0)
        run_to = position_trajectory.end_time()
        simulator.AdvanceTo(run_to)
        visualizer.StopRecording()
        visualizer.PublishRecording()

        output_html.parent.mkdir(parents=True, exist_ok=True)
        output_html.write_text(meshcat.StaticHtml(), encoding="utf-8")
        return output_html


def record_planned_trajectory_to_html(
    position_trajectory: PiecewisePolynomial,
    output_html: Path,
    *,
    duration: float | None = None,
) -> Path:
    planner = PianoRobotPlanner()
    return planner.record_static_html(position_trajectory, output_html, duration=duration)
