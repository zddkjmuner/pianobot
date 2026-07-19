"""Main piano-playing robotic hand project orchestration."""

import numpy as np
import pydot
import pydrake
from IPython.display import SVG, display

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    FindResourceOrThrow,
    FixedOffsetFrame,
    InverseKinematics,
    JacobianWrtVariable,
    LogVectorOutput,
    MeshcatVisualizerCpp,
    MultibodyPlant,
    MultibodyPositionToGeometryPose,
    Parser,
    PiecewisePolynomial,
    PiecewiseQuaternionSlerp,
    PrismaticJoint,
    RevoluteJoint,
    RigidTransform,
    RotationMatrix,
    SceneGraph,
    Simulator,
    Solve,
    TrajectorySource,
)
from pydrake.multibody.tree import (
    DoorHinge,
    DoorHingeConfig,
    LinearSpringDamper,
)

from manipulation import running_as_notebook
from manipulation.meshcat_cpp_utils import (
    MeshcatJointSliders,
    MeshcatPoseSliders,
)
from manipulation.scenarios import AddShape

from pianobot.controller import MyControllerSystem
from pianobot.models import Keyboard_Key
from pianobot.music import generate_note_from_chord
from pianobot.sound import PianoOutputSystem
from pianobot.visualization import get_meshcat


class Piano_Project():
    def __init__(self, sim = None,q_iiwa_traj=None, q_hand_traj=None, q_all_traj=None, w_iiwa_traj=None, w_hand_traj=None,w_all_traj=None,c_hand_traj = None):
        self.builder = DiagramBuilder()
        self.timestep = 1e-5
        
        if sim != "static":
            self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=self.timestep)
        else:
            self.plant = MultibodyPlant(time_step = self.timestep)
            self.scene_graph = self.builder.AddSystem(SceneGraph())
            self.plant.RegisterAsSourceForSceneGraph(self.scene_graph)

        self.sim = sim
        self.mu = 1
        
        self.build_plant()

        self.initialize_plant_before_sim()
        self.initialize_simulation()
        self.build_system(sim, q_iiwa_traj, q_hand_traj,q_all_traj,w_iiwa_traj, w_hand_traj,w_all_traj,c_hand_traj)
        self.setup_plant_after_sim()


            
    # def init_meshcat(self):
    #     self.meshcat = StartMeshcat()
    
    def build_plant(self):
        def build_ground():
            ground = AddShape(self.plant, pydrake.geometry.Box(10,10,2.0), name="ground", mu=self.mu)
            self.plant.WeldFrames(self.plant.world_frame(), self.plant.GetFrameByName("ground"), RigidTransform(p=[0,0,-1.0]))
            self.plant.AddFrame(FixedOffsetFrame("frame_top_ground",
                                self.plant.GetFrameByName("ground"), 
                                RigidTransform(p=[0,0,1])))

        def build_robot():
            def get_robot_file(description):
                # Note: I could download remote model resources here if necessary.
                if description == "Kuka LBR iiwa 7":
                    return FindResourceOrThrow("drake/manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf")
                elif description == "Kuka LBR iiwa 14":
                    return FindResourceOrThrow("drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf")
                elif description == "Kinova Jaco Gen2 (7 DoF)":
                    return FindResourceOrThrow("drake/manipulation/models/jaco_description/urdf/j2s7s300.urdf")
                elif description == "Franka Emika Panda":
                    return FindResourceOrThrow("drake/manipulation/models/franka_description/urdf/panda_arm_hand.urdf")
                raise Exception("Unknown Robot model")

            def get_hand_file(description):
                # Note: I could download remote model resources here if necessary.
                if description == "allegro_hand_description_right":
                    return FindResourceOrThrow("drake/manipulation/models/allegro_hand_description/sdf/allegro_hand_description_right.sdf")
                elif description == "allegro_hand_description_left":
                    return FindResourceOrThrow("drake/manipulation/models/allegro_hand_description/sdf/allegro_hand_description_left.sdf")
                raise Exception("Unknown Hand model")


            robot_arm = "Kuka LBR iiwa 7"
            robot_hand = "allegro_hand_description_right"

            self.model_arm = Parser(self.plant, self.scene_graph).AddModelFromFile(get_robot_file(robot_arm))
            self.plant.WeldFrames(self.plant.GetFrameByName("frame_top_ground"), 
                    self.plant.get_body(self.plant.GetBodyIndices(self.model_arm)[0]).body_frame(),RigidTransform(p=[0,0,0.0001]))
            
            self.hand_frame = self.plant.get_body(
                            self.plant.GetBodyIndices(self.model_arm)[0]).body_frame()

            self.model_hand = Parser(self.plant, self.scene_graph).AddModelFromFile(get_hand_file(robot_hand))

            self.plant.AddFrame(FixedOffsetFrame("frame_palm",
                                self.plant.GetFrameByName("hand_root"), 
                                RigidTransform(p=[0,0,0.05])))

            self.plant.WeldFrames(self.plant.get_body(self.plant.GetBodyIndices(self.model_arm)[7]).body_frame(),
                            self.plant.get_body(self.plant.GetBodyIndices(self.model_hand)[0]).body_frame(),
                            RigidTransform(p=[0,0,0.05]))

            self.finger_link_list = ["link_15", "link_3", "link_7","link_11"]
            self.finger_connect_list = ["link_14", "link_2", "link_6","link_10"]
            
            for body_name in self.finger_link_list:
                self.plant.AddFrame(FixedOffsetFrame("frame_fingercontact_"+body_name,
                        self.plant.GetFrameByName(body_name),
                        RigidTransform(p=[0,0,0.032])))

        def build_piano():
            color_black = [0, 0, 0, 1]
            color_white = [1,1,1,1]
            color_base = [0.3,0.2,0.1,1]           

            self.base_name = "piano_base"
            self.base_size = [1.8, 0.5, 0.2]
            self.base_pos = [-0.20, 0.9, 0]

            key_scale = 2.2
            key_size = np.array([0.019, 0.04, 0.02])*key_scale
            key_offset = np.array([0.0042, 0.0,0])*key_scale

            key_minor_size = key_size
            key_minor_size[0] *=0.75
            key_minor_size[1] *=1

            self.key_size = key_size
            
            keyboard_base = AddShape(self.plant, pydrake.geometry.Box(self.base_size[0],self.base_size[1],self.base_size[2]), name=self.base_name, mass=10, mu=self.mu, color=color_base)
           
            self.plant.AddFrame(FixedOffsetFrame("frame_bottom_"+self.base_name,
                    self.plant.GetFrameByName(self.base_name), 
                    RigidTransform(p=[0,0,-self.base_size[2]*0.5])))

            self.plant.AddFrame(FixedOffsetFrame("frame_top_"+self.base_name,
                                self.plant.GetFrameByName(self.base_name), 
                                RigidTransform(p=[0,0,self.base_size[2]*0.5])))

            self.plant.WeldFrames(self.plant.GetFrameByName("frame_top_ground"), 
                                    self.plant.GetFrameByName("frame_bottom_"+self.base_name), 
                                    RigidTransform(p=self.base_pos))

            def add_key_spring(name,key_pos, key_size,color = color_white):
                key_mass= 0.01
                key_base_name = "key_anchor_"+name
                key_name = "key_"+name
                key_joint_name = "key_joint_"+name

                
                key_base = AddShape(self.plant, 
                                    pydrake.geometry.Box(key_size[0],key_size[1],key_size[2]*0.1), 
                                    name=key_base_name, 
                                    mass=1, 
                                    mu=1, 
                                    color=color)

                self.plant.AddFrame(FixedOffsetFrame("frame_bottom_"+key_base_name,
                                    self.plant.GetFrameByName(key_base_name), 
                                    RigidTransform(p=[0,0,-key_size[2]*0.05])))
                self.plant.AddFrame(FixedOffsetFrame("frame_top_"+key_base_name,
                                    self.plant.GetFrameByName(key_base_name), 
                                    RigidTransform(p=[0,0,+key_size[2]*0.05+0.05])))

                self.plant.WeldFrames(self.plant.GetFrameByName("frame_top_"+self.base_name), 
                                    self.plant.GetFrameByName("frame_bottom_"+key_base_name), 
                                    RigidTransform(p=key_pos))
                
                
                key = AddShape(self.plant, 
                                    pydrake.geometry.Box(key_size[0],key_size[1],key_size[2]), 
                                    name=key_name, 
                                    mass=key_mass, 
                                    mu=self.mu, 
                                    color=color)

                self.plant.AddFrame(FixedOffsetFrame("frame_bottom_"+key_name,
                                    self.plant.GetFrameByName(key_name), 
                                    RigidTransform(p=[0,0,-key_size[2]*0.5])))

                X_KT = RigidTransform(p=[0,0,+key_size[2]*0.3])
                X_KR1 = RigidTransform(R=RotationMatrix.MakeYRotation(np.pi))
                X_KR2 = RigidTransform(R=RotationMatrix.MakeZRotation(-np.pi/2))
                X_FContact = X_KT.multiply(X_KR1.multiply(X_KR2))

                self.plant.AddFrame(FixedOffsetFrame("frame_keycontact_"+key_name,
                                    self.plant.GetFrameByName(key_name),
                                    X_FContact))
                                    
                key_Joint = self.plant.AddJoint(
                                    PrismaticJoint(name = key_joint_name,
                                    frame_on_parent=self.plant.GetFrameByName("frame_top_"+key_base_name),
                                    frame_on_child=self.plant.GetFrameByName("frame_bottom_"+key_name),
                                    axis=[0,0,1],
                                    pos_upper_limit = 0.1,
                                    pos_lower_limit = 0.01,
                                    damping=0))

                key_spring = self.plant.AddForceElement(
                            LinearSpringDamper( bodyA = self.plant.GetBodyByName(key_base_name),
                                                p_AP = [0,0,key_size[2]*0.1*0.5],
                                                bodyB = self.plant.GetBodyByName(key_name),
                                                p_BQ = [0,0,-key_size[2]*0.5],
                                                free_length = 0.03,
                                                stiffness = 0.01,
                                                damping = 0.0001)            )    
                return

            def add_key_hinge(name,key_pos, key_size,color = color_white):
                key_mass= 0.1
                key_base_name = "key_anchor_"+name
                key_name = "key_"+name
                key_joint_name = "key_joint_"+name

                key_moment_arm = key_size[1]*7
                color_key_base = [0.4,0.2,0.1,1]
                key_base = AddShape(self.plant, 
                                    pydrake.geometry.Box(key_size[0],key_size[0]*0.5,key_size[2]*0.1), 
                                    name=key_base_name, 
                                    mass=1, 
                                    mu=1, 
                                    color=color_key_base)

                self.plant.AddFrame(FixedOffsetFrame("frame_bottom_"+key_base_name,
                                    self.plant.GetFrameByName(key_base_name), 
                                    RigidTransform(p=[0,0,-key_size[2]*0.05])))
                self.plant.AddFrame(FixedOffsetFrame("frame_top_"+key_base_name,
                                    self.plant.GetFrameByName(key_base_name), 
                                    RigidTransform(p=[0,0,+key_size[2]*0.05+0.05])))

                hinge_offset_constant = np.array([0, key_moment_arm ,key_size[2]*2])
                p_key_hinge = RigidTransform(p=hinge_offset_constant+key_pos)

                self.plant.WeldFrames(self.plant.GetFrameByName("frame_top_"+self.base_name), 
                                    self.plant.GetFrameByName("frame_bottom_"+key_base_name), 
                                    p_key_hinge)
                
                
                key = AddShape(self.plant, 
                                    pydrake.geometry.Box(key_size[0],key_size[1],key_size[2]), 
                                    name=key_name, 
                                    mass=key_mass, 
                                    mu=self.mu, 
                                    color=color)

                self.plant.AddFrame(FixedOffsetFrame("frame_rev_"+key_name,
                                    self.plant.GetFrameByName(key_name), 
                                    RigidTransform(p=[0,key_moment_arm,0])))

                X_KT = RigidTransform(p=[0,0,key_size[2]*0.25])
                X_KR1 = RigidTransform(R=RotationMatrix.MakeYRotation(np.pi))
                X_KR2 = RigidTransform(R=RotationMatrix.MakeZRotation(-np.pi/2))
                X_FContact = X_KT.multiply(X_KR1.multiply(X_KR2))

                self.plant.AddFrame(FixedOffsetFrame("frame_keycontact_"+key_name,
                                    self.plant.GetFrameByName(key_name),
                                    X_FContact))
                        
                key_revolute_joint = self.plant.AddJoint(RevoluteJoint(
                        name="revolve_joint_"+key_name, 
                        frame_on_parent=self.plant.GetFrameByName(key_base_name),
                        frame_on_child=self.plant.GetFrameByName("frame_rev_"+key_name), 
                        axis=[1, 0, 0],
                        damping=0.0))    

                door_hinge_config = DoorHingeConfig()
                door_hinge_config.spring_constant = 5
                door_hinge_config.spring_zero_angle_rad=1#key_size[2]/key_moment_arm*1.1
                door_hinge_config.catch_torque =door_hinge_config.spring_constant*1.55#door_hinge_config.spring_constant #door_hinge_config.spring_constant
                door_hinge_config.catch_width = door_hinge_config.spring_zero_angle_rad#1#door_hinge_config.spring_zero_angle_rad #door_hinge_config.spring_zero_angle_rad*0.9
                
                door_hinge_config.viscous_friction  = door_hinge_config.spring_constant*0.2#door_hinge_config.spring_constant*0.1
                door_hinge_config.static_friction_torque =door_hinge_config.spring_constant*0.01#0.01
                door_hinge_config.dynamic_friction_torque = door_hinge_config.spring_constant*0.0#door_hinge_config.spring_constant*0.01
                door_hinge = self.plant.AddForceElement(DoorHinge(
                        joint=key_revolute_joint, config=door_hinge_config))
                return


            key_major_list = ["C","D","E","F","G","A","B"]
            key_minor_list = ["C#","D#","E#","F#","G#","A#","B#"]
            N_scale = 3

            key_major_pos = np.array([-(key_size[0]+key_offset[0])*(7*N_scale/2)*2+0.2,-self.base_size[1]*0.5,0])
            
            self.key_list =[]
            N_scale += 1 # Add one more octave 
            N_scale += 1 # Add one more octave 

            for s in range(N_scale):
                for i in range(7):
                    key_name = key_major_list[i]+"_{}".format(s)
                    # add_key_spring(key_name, key_major_pos, key_size)
                    add_key_hinge(key_name, key_major_pos, key_size)
                    self.key_list.append(key_name)

                    if i==0 or i==1 or i==3 or i==4 or i==5:
                        key_minor_name = key_minor_list[i]+"_{}".format(s)
                        key_minor_pos = np.copy(key_major_pos)
                        key_minor_pos[0] += (key_size[0]+key_offset[0])*0.5
                        key_minor_pos[1] += (key_size[1]+0.005)
                        key_minor_pos[2] += key_size[2]*0.5

                        add_key_hinge(key_minor_name, key_minor_pos, key_minor_size,color=color_black)
                        
                        self.key_list.append(key_minor_name)
                        

                    key_major_pos[0] += key_size[0]+key_offset[0]
            return

        build_ground()
        build_piano()
        build_robot()
        self.plant.mutable_gravity_field().set_gravity_vector([0, 0, 0]) # easy trick for gravity compensation
        self.plant.Finalize()

    def initialize_plant_before_sim(self):
        def initialize_robot():
            # self.finger_link_list = ["link_11", "link_7", "link_3","link_15"]
            # self.connect_link_list = ["link_14", "link_2", "link_6","link_10"]


            # def add_triad(link_names):
            #     for body_name in link_names:
            #         AddMultibodyTriad(self.plant.GetFrameByName(body_name), self.scene_graph,length=0.05)

            # add_triad(self.finger_link_list)            
            self.fingers = []
            self.finger_frames = []
            self.finger_link_frames = []

            self.palm_frame = self.plant.GetFrameByName("frame_palm")
            # AddMultibodyTriad(self.palm_frame , self.scene_graph,length=0.05)
            
            for body_name in self.finger_link_list:
                self.fingers.append(self.plant.GetBodyByName(body_name))
                # self.finger_frames.append(self.plant.GetFrameByName(body_name))
                self.finger_frames.append(self.plant.GetFrameByName("frame_fingercontact_"+body_name))
                # AddMultibodyTriad(self.plant.GetFrameByName("frame_fingercontact_"+body_name), self.scene_graph,length=0.01)

            for body_name in self.finger_connect_list:
                self.finger_link_frames.append(self.plant.GetFrameByName(body_name))
                # AddMultibodyTriad(self.plant.GetFrameByName(body_name,self.model_hand), self.scene_graph,length=0.05)
        
        def initialize_piano():
            for key in self.key_list:
                frame_name = "frame_keycontact_key_"+key
                key_frame = self.plant.GetFrameByName(frame_name)
                #ddMultibodyTriad(key_frame , self.scene_graph,length=0.05)


        initialize_robot()
        initialize_piano()

    def setup_plant_after_sim(self):
        def setup_piano():
            for key in self.key_list:
                frame_name = "frame_keycontact_key_"+key
                key_frame = self.plant.GetFrameByName(frame_name)

                key_pose = self.plant.CalcRelativeTransform(
                            self.plant_context,
                            frame_A=self.plant.GetFrameByName("frame_top_ground"),
                            frame_B=key_frame)
                            
                # AddMeshcatTriad(get_meshcat(), "meshcat_" + frame_name,
                #             length=0.015, radius=0.006, X_PT=key_pose)
                            

            self.note_dictionary =dict.fromkeys(self.key_list)
            # self.music_note_list = []

            for idx, key in enumerate(self.key_list):
                key_pose = self.plant.CalcRelativeTransform(
                            self.plant_context,
                            frame_A=self.plant.GetFrameByName("frame_top_ground"),
                            frame_B=self.plant.GetFrameByName("frame_keycontact_key_"+key))
                pos = key_pose.translation()
                rotation = key_pose.rotation()

                key_frame = self.plant.GetFrameByName("frame_keycontact_key_"+key)
                
                k_type = idx%12 
                if k_type == 1 or k_type ==3 or k_type ==6 or k_type ==8 or k_type==10:
                    key_type = "sharp"
                else:
                    key_type = "flat"
            
                self.note_dictionary[key] = Keyboard_Key(key, pos=pos,rotation = rotation,frame = key_frame,keytype =key_type)
            


            first_key = self.key_list[0]
            first_key_pose = self.plant.CalcRelativeTransform(
                            self.plant_context,
                            frame_A=self.plant.GetFrameByName("frame_top_ground"),
                            frame_B=self.plant.GetFrameByName("key_"+first_key))
            first_key_pos = first_key_pose.translation()
            
            last_key = self.key_list[-1]
            last_key_pose = self.plant.CalcRelativeTransform(
                            self.plant_context,
                            frame_A=self.plant.GetFrameByName("frame_top_ground"),
                            frame_B=self.plant.GetFrameByName("key_"+last_key))
            last_key_pos = first_key_pose.translation()
            
            self.music_note_list = list(self.note_dictionary)

            return

        if self.sim != "static":
            setup_piano()       


    def get_finger_pose(self, index=0):
        pose = RigidTransform()
        
        if index <4:
            pose = self.plant.EvalBodyPoseInWorld(self.plant_context, self.fingers[index])
        
        return pose

    def initialize_simulation(self):
        self.visualizer = MeshcatVisualizerCpp.AddToBuilder(self.builder, self.scene_graph, get_meshcat())
        
        
    def build_system(self, sim = None, q_iiwa_traj=None, q_hand_traj=None, q_all_traj = None, w_iiwa_traj=None, w_hand_traj=None, w_all_traj=None, c_hand_traj=None):

        def connect_controller():
            self.robot_controller = self.builder.AddSystem(MyControllerSystem(
                                                        self.plant))

            self.builder.Connect(self.plant.GetOutputPort("iiwa7_continuous_state"),
                                    self.robot_controller.GetInputPort("iiwa7_state"))
            self.builder.Connect(self.plant.GetOutputPort("allegro_hand_right_continuous_state"),
                                    self.robot_controller.GetInputPort("hand_state"))

            # self.builder.Connect(self.plant.GetOutputPort("allegro_hand_right_generalized_contact_forces"),
            #                         self.robot_controller.GetInputPort("hand_contact_forces"))
                                    
            # self.builder.Connect(self.plant.GetOutputPort("contact_results"),
            #                     self.robot_controller.GetInputPort("contact_result"))

            self.builder.Connect(self.robot_controller.GetOutputPort("iiwa7_torque"),
                                    self.plant.GetInputPort("iiwa7_actuation"))
            self.builder.Connect(self.robot_controller.GetOutputPort("hand_torque"),
                                    self.plant.GetInputPort("allegro_hand_right_actuation"))
            
        

        def add_trajectory_to_controller(q_iiwa_traj, q_hand_traj,w_iiwa_traj,w_hand_traj,c_hand_traj):
            q_iiwa_source = self.builder.AddSystem(TrajectorySource(q_iiwa_traj))
            q_iiwa_source.set_name("q_iiwa_traj")
            q_hand_source = self.builder.AddSystem(TrajectorySource(q_hand_traj))
            q_hand_source.set_name("q_hand_traj")
            
            self.builder.Connect(q_iiwa_source.get_output_port(),
                                    self.robot_controller.GetInputPort("iiwa7_des_pos"))
            self.builder.Connect(q_hand_source.get_output_port(),
                                    self.robot_controller.GetInputPort("hand_des_pos"))


            w_iiwa_source = self.builder.AddSystem(TrajectorySource(w_iiwa_traj))
            w_iiwa_source.set_name("w_iiwa_traj")
            self.builder.Connect(w_iiwa_source.get_output_port(),
                    self.robot_controller.GetInputPort("iiwa7_des_vel"))


            w_hand_source = self.builder.AddSystem(TrajectorySource(w_hand_traj))
            w_hand_source.set_name("w_hand_traj")
            self.builder.Connect(w_hand_source.get_output_port(),
                    self.robot_controller.GetInputPort("hand_des_vel"))

            c_hand_source = self.builder.AddSystem(TrajectorySource(c_hand_traj))
            c_hand_source.set_name("c_hand_traj")
            self.builder.Connect(c_hand_source.get_output_port(),
                    self.robot_controller.GetInputPort("finger_des_contact"))


        def add_geomtery_generator(q_all_traj):
            q_all_source = self.builder.AddSystem(TrajectorySource(q_all_traj))
            q_all_source.set_name("q_all_source")
            
            w_all_source = self.builder.AddSystem(TrajectorySource(w_all_traj))
            w_all_source.set_name("w_all_source")


            self.piano_sound = self.builder.AddSystem(PianoOutputSystem(self.plant,self.key_list))
            self.builder.Connect(q_all_source.get_output_port(), self.piano_sound.GetInputPort("q"))
            self.builder.Connect(w_all_source.get_output_port(), self.piano_sound.GetInputPort("w"))
            
            self.piano_sound.set_name("piano_sound_generator")
            
            self.robot_pose = self.builder.AddSystem(MultibodyPositionToGeometryPose(self.plant))

            self.builder.Connect(self.piano_sound.GetOutputPort("state_output"), self.robot_pose.get_input_port())
            self.builder.Connect(   self.robot_pose.get_output_port(),
                                self.scene_graph.get_source_pose_port(self.plant.get_source_id()))


        def add_data_logger():
                self.q_logger = LogVectorOutput(self.plant.GetOutputPort("continuous_state"), self.builder)
                self.q_iiwa_logger = LogVectorOutput(self.plant.GetOutputPort("iiwa7_continuous_state"), self.builder)
                self.q_hand_logger = LogVectorOutput(self.plant.GetOutputPort("allegro_hand_right_continuous_state"), self.builder)
                
        if sim == "dynamic":
            connect_controller()
            add_trajectory_to_controller(q_iiwa_traj, q_hand_traj,w_iiwa_traj,w_hand_traj,c_hand_traj)
            add_data_logger()

        elif sim == "static":
            add_geomtery_generator(q_all_traj)

        else: #sim is None
            connect_controller()

        self.diagram = self.builder.Build()
        self.context = self.diagram.CreateDefaultContext()
        self.plant_context =  self.plant.GetMyMutableContextFromRoot(self.context)

    def forward_kinematics_demo(self):
        def my_callback(context):

            self._iiwa = self.plant.GetModelInstanceByName("iiwa7")
            self._hand = self.plant.GetModelInstanceByName("allegro_hand_right")
            self.iiwa_indices = self.plant.GetJointIndices(self._iiwa)
            self.hand_indices = self.plant.GetJointIndices(self._hand)

            qqq = self.plant.GetOutputPort("continuous_state").Eval(self.plant_context)

            q =self.plant.GetPositions(self.plant_context)
            dq =self.plant.GetPositions(self.plant_context)

            self._W = self.plant.world_frame()

            finger_name_list = ["link_15", "link_3", "link_7","link_11"]
            _F0 = self.plant.GetFrameByName("frame_fingercontact_"+finger_name_list[0])
            _P = self.plant.GetFrameByName("frame_palm")
            _W = self.plant.world_frame()

            J_G = self.plant.CalcJacobianSpatialVelocity(
                self.plant_context, JacobianWrtVariable.kV, 
                _F0, [0,0,0], _W, _W)

            print(np.shape(qqq))
            print(np.shape(J_G))
            clear_output(wait=True)
        sliders = MeshcatJointSliders(get_meshcat(), self.plant, self.context)
        sliders.Run(self.visualizer, self.context,my_callback)
        get_meshcat().DeleteAddedControls()
        get_meshcat().Delete()
    
    def run_simulation_demo(self, q0_init=np.zeros((71+12,1)),play_rate =1,t_duration = 40):
        
        hand_indices =np.array([43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58])+12+12
        
        q0_init[hand_indices] = 0

        if self.sim =="dynamic":
            q0 = self.plant.GetPositions(self.plant_context)
            _,_,_,q_all = self.ik_init(q0)
            self.plant.SetPositions(self.plant_context, q_all)
            self.plant.SetPositions(self.plant_context, q0_init)

        
        simulator = Simulator(self.diagram, self.context)
        context = simulator.get_mutable_context()

        simulator.set_target_realtime_rate(play_rate)
        duration = t_duration if running_as_notebook else 0.01
        simulator.AdvanceTo(duration)

        get_meshcat().DeleteAddedControls()
        get_meshcat().Delete()
        
        if self.sim =="dynamic":
            log = self.q_logger.FindMutableLog(context)
            return log
    

    def inverse_kinematics_demo(self):

        q0 = self.plant.GetPositions(self.plant_context)
        finger_frame = self.plant.GetFrameByName("link_7",self.model_hand)

        def my_callback(context, pose):
            ik = InverseKinematics(self.plant, self.plant_context)
            ik.AddPositionConstraint(
                finger_frame, [0, 0, 0], self.plant.world_frame(),
                pose.translation(), pose.translation())
            # ik.AddOrientationConstraint(
            #     gripper_frame, RotationMatrix(), plant.world_frame(),
            #     pose.rotation(), 0.0)
            # ik.AddMinimumDistanceConstraint(0.001, 0.1)
            prog = ik.get_mutable_prog()
            q = ik.q()
            # prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
            prog.SetInitialGuess(q, q0)
            result = Solve(ik.prog())
            
            # clear_output(wait=True)

            # if result.is_success():
            #     print("IK success")
            # else:
            #     print("IK failure")

        sliders = MeshcatPoseSliders(get_meshcat())
        sliders.SetPose(self.plant.EvalBodyPoseInWorld(
            self.plant_context, self.plant.GetBodyByName("link_7",self.model_hand)))

        # set the initial z lower, to make the interaction interesting.
        # sliders.SetXyz([0.4, 0.2, 0.65])
        sliders.Run(self.visualizer, self.context, my_callback)
        get_meshcat().DeleteAddedControls()
        get_meshcat().Delete()

        return
    def visualize_kinematic_tree(self):  
        display(SVG(pydot.graph_from_dot_data(self.plant.GetTopologyGraphvizString())[0].create_svg()))

    def visualize_diagram(self):  
        display(SVG(pydot.graph_from_dot_data(self.diagram.GetGraphvizString())[0].create_svg()))
    
    def test_finger_planning(self):
        tchord = [self.music_note_list[0],self.music_note_list[4], self.music_note_list[7]]

        test_music_notes = [self.note_dictionary[tchord[0]], self.note_dictionary[tchord[1]], self.note_dictionary[tchord[2]]]

        n, fidx,cost = self.find_finger_correspondence(test_music_notes)
        print(self.find_finger_correspondence(test_music_notes))

    def find_finger_correspondence(self,key_list):
        N_finger =4
        f_pos = self.get_finger_pose(0).translation()
        for i in range(N_finger-1):
            f_pos = np.vstack((f_pos,self.get_finger_pose(i+1).translation()))

        N_key = len(key_list)
        indices = np.empty(N_key,dtype = int)
        
        if N_key >= 4:
            fidx = np.zeros((1,4))
            cost = np.zeros((1,1))
            
            N_sol = 1
            fidx[0,:] = [0,1,2,3]
            cost[0,0] = 1

        elif N_key <=0:
            N_sol = 0
            fidx = -1
            cost = -1

        else:
            dist =np.zeros((N_key, N_finger))
            for key_idx,key in enumerate(key_list):
                for f in range(N_finger):
                    note = self.note_dictionary[key]
                    p_FK = note.pos - self.get_finger_pose(f).translation()
                    d_FK = np.dot(p_FK[:2], p_FK[:2]) # only consider x and y 
                    dist[key_idx,f] = d_FK

            if N_key ==1: 
                N_sol = 3
                fidx = np.zeros((N_sol,N_key))
                cost = np.zeros((N_sol,1))   
                i_sol = 0                 
                for f in range(1,N_finger):
                    fidx[i_sol,0] = f
                    cost[i_sol,0] = dist[0,f]
                    i_sol+=1

            elif N_key ==2:
                N_sol = 6
                fidx = np.zeros((N_sol,N_key))
                cost = np.zeros((N_sol,1))    

                i_sol = 0
                for f1 in range(N_finger):
                    for f2 in range(f1+1,N_finger):
                        fidx[i_sol,:] = [f1,f2]
                        cost[i_sol,0] = dist[0,f1]+dist[0,f2]
                        i_sol+=1

            elif N_key ==3:
                N_sol = 4
                fidx = np.zeros((N_sol,N_key))
                cost = np.zeros((N_sol,1))    

                i_sol = 0
                for f1 in range(N_finger):
                    for f2 in range(f1+1,N_finger):
                        for f3 in range(f2+1,N_finger):
                            fidx[i_sol,:] = [f1,f2,f3]
                            cost[i_sol,0] = dist[0,f1]+dist[0,f2]+dist[0,f3]
                            i_sol+=1                                
            
            

        return N_sol, fidx, cost

    def ik_init(self, q0):

        ik = InverseKinematics(self.plant, self.plant_context)

        T_center = self.note_dictionary[self.key_list[18]].pos

        T_const_palm_lower = np.array([-0.01,-self.key_size[1]*4-0.01,self.key_size[2]*3])
        T_const_palm_upper = np.array([+0.01,-self.key_size[1]*2+0.01,self.key_size[2]*5+0.01])

        T_palm_lower =T_center + T_const_palm_lower
        T_palm_upper =T_center + T_const_palm_upper


        c = ik.AddPositionConstraint(
            self.palm_frame , [0, 0, 0], self.plant.world_frame(),
            T_palm_lower, T_palm_upper)

        c = ik.AddAngleBetweenVectorsConstraint(
            self.palm_frame, [0, 0, 1],
            self.plant.world_frame(), [0,1,0],
            0, np.pi/2*0.05
        )
        c = ik.AddAngleBetweenVectorsConstraint(
            self.palm_frame, [1, 0, 0],
            self.plant.world_frame(), [0,0,-1],
            0, np.pi/2*0.05
        )               
        c = ik.AddAngleBetweenVectorsConstraint(
            self.palm_frame, [0, 1, 0],
            self.plant.world_frame(), [-1,0,0],
            0, np.pi/2*0.05
        )               
           
        prog = ik.get_mutable_prog()
        q = ik.q()
        prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
        prog.SetInitialGuess(q, q0)
        result = Solve(ik.prog())

        q_iiwa = self.plant.GetPositions(self.plant_context, 
                    self.plant.GetModelInstanceByName("iiwa7"))
        q_hand = self.plant.GetPositions(self.plant_context, 
                    self.plant.GetModelInstanceByName("allegro_hand_right"))  

        q_all = result.GetSolution()
        return result.is_success(), q_iiwa, q_hand, q_all

    def ik_prepress_chord(self, q0, note_on_fingers):

        ik = InverseKinematics(self.plant, self.plant_context)

        T_finger = []
        T_link = []

        for i in range(4):
            T_finger.append( self.plant.CalcRelativeTransform(
                                self.plant_context,
                                frame_A=self.plant.GetFrameByName("frame_top_ground"),
                                frame_B=self.finger_frames[i]).translation())

            T_link.append( self.plant.CalcRelativeTransform(
                                self.plant_context,
                                frame_A=self.plant.GetFrameByName("frame_top_ground"),
                                frame_B=self.finger_link_frames[i]).translation())

        T_palm = np.copy(self.plant.CalcRelativeTransform(
                                self.plant_context,
                                frame_A=self.plant.GetFrameByName("frame_top_ground"),
                                frame_B=self.palm_frame).translation())
                                
        T_const_finger_tip_offset = np.array([0,0,self.key_size[2]*1.6])

        T_const_fingertip_flat_lower = np.array([-self.key_size[0]*0.15,-self.key_size[1]*0.4,-self.key_size[2]*0.01])
        T_const_fingertip_flat_upper = np.array([+self.key_size[0]*0.15,+self.key_size[1]*0.15,+self.key_size[2]])

        T_const_fingertip_sharp_lower = np.array([-self.key_size[0]*0.15,-self.key_size[1]*0.3,-self.key_size[2]*0.01])
        T_const_fingertip_sharp_upper = np.array([+self.key_size[0]*0.15,+self.key_size[1]*0.2,+self.key_size[2]])

        T_const_fingertip_none_lower = np.array([-self.key_size[0],-self.key_size[1]*0.5,self.key_size[2]*0.01])
        T_const_fingertip_none_upper = np.array([+self.key_size[0],+self.key_size[1]*0.5,+self.key_size[2]*1.5])
        
        finger_cnt = 0



        T_const_fingertip_flat_lower2 = np.array([-self.key_size[0]*0.15,-self.key_size[1]*0.6,-self.key_size[2]*0.5])
        T_const_fingertip_flat_upper2 = np.array([+self.key_size[0]*0.3,+self.key_size[1]*0.1,+self.key_size[2]*2])

        T_const_fingertip_sharp_lower2 = np.array([-self.key_size[0]*0.3,-self.key_size[1]*0.5,-self.key_size[2]*0.5])
        T_const_fingertip_sharp_upper2 = np.array([+self.key_size[0]*0.3,+self.key_size[1]*0.3,+self.key_size[2]*2])


        for idx, note in enumerate(note_on_fingers):
            T_key = np.copy(T_finger[idx])
            T_keylink = np.copy(T_link[idx])

            
            if note != None and "none" in note:
                
                T_lower = T_key + T_const_fingertip_none_lower
                T_upper = T_key + T_const_fingertip_none_upper

                T_link_lower = T_keylink + T_const_fingertip_none_lower
                T_linK_upper = T_keylink + T_const_fingertip_none_upper
            else:
                key_type = self.note_dictionary[note].keytype

                T_key2 = np.copy(self.note_dictionary[note].pos)
                
                if key_type =="flat":
                    T_key += T_const_finger_tip_offset
                    T_keylink += T_const_finger_tip_offset

                    T_lower = T_key + T_const_fingertip_flat_lower
                    T_upper = T_key + T_const_fingertip_flat_upper

                    T_link_lower = T_keylink + T_const_fingertip_none_lower
                    T_linK_upper = T_keylink + T_const_fingertip_none_upper


                    # T_key2 += T_const_finger_tip_offset
                    # T_lower2 = T_key2 + T_const_fingertip_flat_lower2
                    # T_upper2 = T_key2 + T_const_fingertip_flat_upper2

                else :
                    # T_key = self.note_dictionary[note].pos
                    T_key += T_const_finger_tip_offset
                    T_keylink += T_const_finger_tip_offset
                    T_lower = T_key + T_const_fingertip_sharp_lower
                    T_upper = T_key + T_const_fingertip_sharp_upper
                    T_link_lower = T_keylink + T_const_fingertip_none_lower
                    T_linK_upper = T_keylink + T_const_fingertip_none_upper
                
                    # T_key2 += T_const_finger_tip_offset
                    # T_lower2 = T_key2 + T_const_fingertip_sharp_lower2
                    # T_upper2 = T_key2 + T_const_fingertip_sharp_upper2

      
            c = ik.AddPositionConstraint(
                self.finger_frames[idx], [0, 0, 0],self.plant.world_frame(),
                T_lower, T_upper)

      
            # c = ik.AddPositionConstraint(
            #     self.finger_frames[idx], [0, 0, 0],self.plant.world_frame(),
            #     T_lower2, T_upper2)
                
            c = ik.AddPositionConstraint(
                self.finger_link_frames[idx], [0, 0, 0],self.plant.world_frame(),
                T_link_lower, T_linK_upper)

            # if "none" in note:
            #     1

            # else:
            #     if key_type == "flat":
            #         c = ik.AddAngleBetweenVectorsConstraint(
            #             self.finger_frames[idx], [0, 0, 1],
            #             self.plant.world_frame(), [0,0,-1],
            #             0, np.pi/2*0.3
            #         )
            #         c = ik.AddAngleBetweenVectorsConstraint(
            #             self.finger_frames[idx], [0, 0, 1],
            #             self.plant.world_frame(), [1,0,0],
            #             np.pi/2*0.95, np.pi/2*1.05
            #         )

            #         c = ik.AddAngleBetweenVectorsConstraint(
            #             self.finger_frames[idx], [0, 0, 1],
            #             self.plant.world_frame(), [1,0,0],
            #             np.pi/2*0.95, np.pi/2*1.3
            #         )
            #     else:
            #         c = ik.AddAngleBetweenVectorsConstraint(
            #             self.finger_frames[idx], [0, 0, 1],
            #             self.plant.world_frame(), [0,0,-1],
            #             0, np.pi/2*0.7
            #         )
            #         c = ik.AddAngleBetweenVectorsConstraint(
            #             self.finger_frames[idx], [0, 0, 1],
            #             self.plant.world_frame(), [1,0,0],
            #             np.pi/2*0.8, np.pi/2*1.2
            #         )

                    # c = ik.AddAngleBetweenVectorsConstraint(
                    #     self.finger_frames[idx], [0, 0, 1],
                    #     self.plant.world_frame(), [1,0,0],
                    #     np.pi/2*0.95, np.pi/2*1.3       
                    #              

        T_const_palm_lower = np.array([-self.key_size[0]*2,-self.key_size[1]*0.4,-self.key_size[2]*0.3])
        T_const_palm_upper = np.array([+self.key_size[0]*2,+self.key_size[1]*0.45,+self.key_size[2]*2.5])

        T_palm += T_const_finger_tip_offset
        T_palm_lower = T_palm + T_const_palm_lower
        T_palm_upper = T_palm + T_const_palm_upper

        c = ik.AddPositionConstraint(
            self.palm_frame , [0, 0, 0], self.plant.world_frame(),
            T_palm_lower, T_palm_upper)

        c = ik.AddAngleBetweenVectorsConstraint(
            self.palm_frame, [0, 0, 1],
            self.plant.world_frame(), [0,1,0],
            0, np.pi/2*0.67
        )
        # T_const_hand_lower = np.array([-2,-2,self.key_size[2]*0.2])
        # T_const_hand_upper = np.array([+2,2,2])
        # T_hand_lower = T_center + T_const_hand_lower
        # T_hand_upper = T_center + T_const_hand_upper

        # c = ik.AddPositionConstraint(
        #     self.hand_frame, [0, 0, 0],self.plant.world_frame(),
        #     T_hand_lower, T_palm_upper)
                
        prog = ik.get_mutable_prog()
        q = ik.q()
        prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
        prog.SetInitialGuess(q, q0)
        result = Solve(ik.prog())

        q_iiwa = self.plant.GetPositions(self.plant_context, 
                    self.plant.GetModelInstanceByName("iiwa7"))
        q_hand = self.plant.GetPositions(self.plant_context, 
                    self.plant.GetModelInstanceByName("allegro_hand_right"))  

        return result.is_success(), q_iiwa, q_hand

    def ik_transition(self, q0):

        ik = InverseKinematics(self.plant, self.plant_context)

        T_center = self.note_dictionary[self.key_list[18]].pos
        T_const_release_lower = np.array([-1.3,-1, self.key_size[2]*1.1])
        T_const_release_upper = np.array([1.3,1,self.key_size[2]*10])


        for idx in range(4):
            T_key = T_center
            T_lower = T_key + T_const_release_lower
            T_upper = T_key + T_const_release_upper


            c = ik.AddPositionConstraint(
                self.finger_frames[idx], [0, 0, 0],self.plant.world_frame(),
                T_lower, T_upper)


        T_const_palm_rest_lower = np.array([-1.3,-1,self.key_size[2]*1.2])
        T_const_palm_rest_upper = np.array([+1.3,1,self.key_size[2]*10])

        T_palm_lower =T_key + T_const_palm_rest_lower
        T_palm_upper =T_key + T_const_palm_rest_upper

        c = ik.AddPositionConstraint(
            self.palm_frame , [0, 0, 0], self.plant.world_frame(),
            T_palm_lower, T_palm_upper)

        c = ik.AddAngleBetweenVectorsConstraint(
            self.palm_frame, [0, 0, 1],
            self.plant.world_frame(), [0,1,0],
            0, np.pi/2*0.5
        )
        # T_const_hand_lower = np.array([-2,-2,self.key_size[2]*0.2])
        # T_const_hand_upper = np.array([+2,2,2])
        # T_hand_lower = T_center + T_const_hand_lower
        # T_hand_upper = T_center + T_const_hand_upper

        # c = ik.AddPositionConstraint(
        #     self.hand_frame, [0, 0, 0],self.plant.world_frame(),
        #     T_hand_lower, T_palm_upper)
                
        prog = ik.get_mutable_prog()
        q = ik.q()
        prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
        prog.SetInitialGuess(q, q0)
        result = Solve(ik.prog())

        q_iiwa = self.plant.GetPositions(self.plant_context, 
                    self.plant.GetModelInstanceByName("iiwa7"))
        q_hand = self.plant.GetPositions(self.plant_context, 
                    self.plant.GetModelInstanceByName("allegro_hand_right"))  

        return result.is_success(), q_iiwa, q_hand

    def ik_transition_arm(self, q_start, q_end, N_list=1):

        ik = InverseKinematics(self.plant, self.plant_context)
        

        def get_pose(frame):
            return self.plant.CalcRelativeTransform(
                                self.plant_context,
                                frame_A=self.plant.GetFrameByName("frame_top_ground"),
                                frame_B=frame)

        self.plant.SetPositions(self.plant_context, q_start)
        X_start_palm = get_pose(self.palm_frame)
        T_start_palm = X_start_palm.translation()
        R_start_palm = X_start_palm.rotation()

        self.plant.SetPositions(self.plant_context, q_end)
        X_end_palm = get_pose(self.palm_frame)
        T_end_palm = X_end_palm.translation()
        R_end_palm = X_end_palm.rotation()
        

        palm_T_traj = PiecewisePolynomial.FirstOrderHold(
            [0.0, 1.0],
            np.vstack([[T_start_palm],
                        [T_end_palm]]).T)

        palm_R_traj = PiecewiseQuaternionSlerp()
        palm_R_traj.Append(0.0,  R_start_palm)
        palm_R_traj.Append(1.0, R_end_palm)
        
        n = N_list
        q0s = q_start
        q0e = q_end

        t_sol = []
        q_iiwa_sol = []
        q_hand_sol = []
        q_sol = []
        for i in range(N_list):
            t = float(i+1)/(N_list+1)

            if i%2 ==0: # From start
                t_now = (1)/(n+1)
                T_now = palm_T_traj.value(t_now)
                T_end = palm_T_traj.value(1)
                R_now = palm_R_traj.value(t_now)
                R_end = palm_R_traj.value(1)
                q0 = q0s*(1-t_now) +q0e*t_now

            else: # from end
                t_now = float((n)/(n+1))
                T_start = palm_T_traj.value(0)
                R_start = palm_R_traj.value(0)
                T_now = palm_T_traj.value(t_now)
                R_now = palm_R_traj.value(t_now)
                q0 = q0s*(1-t_now) +q0e*t_now

            T_offset = np.array([[0.001], [0.001], [0.001]])
            c = ik.AddPositionConstraint(
                self.palm_frame , [0, 0, 0], self.plant.world_frame(),
                T_now-T_offset, T_now+T_offset)


            c2=ik.AddOrientationConstraint(
                frameAbar = self.palm_frame, 
                R_AbarA = RotationMatrix(), 
                frameBbar = self.plant.world_frame(),
                R_BbarB = RotationMatrix(R_now), 
                theta_bound = 0.8)
            
            prog = ik.get_mutable_prog()
            q = ik.q()

            prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
            prog.SetInitialGuess(q, q0)
            result = Solve(ik.prog())

            q = self.plant.GetPositions(self.plant_context)
            
            q_iiwa = self.plant.GetPositions(self.plant_context, 
                        self.plant.GetModelInstanceByName("iiwa7"))
            q_hand = self.plant.GetPositions(self.plant_context, 
                        self.plant.GetModelInstanceByName("allegro_hand_right"))  

            X_palm = get_pose(self.palm_frame)
            T_palm = X_start_palm.translation()
            R_palm = X_start_palm.rotation()

            if i%2 ==0: # From start
                q0s  = q
                T_start = T_palm
                R_start = R_palm
            else:
                q0e  = q
                T_end = T_palm
                R_end = R_palm
            
            
            t_sol.append(t)
            q_iiwa_sol.append(q_iiwa)
            q_hand_sol.append(q_hand)
            q_sol.append(q)
            n-=1

            if n!=0:

                palm_T_traj = PiecewisePolynomial.FirstOrderHold(
                    [0.0, 1.0],
                    np.vstack([[np.squeeze(T_start_palm)],
                                [np.squeeze(T_end)]]).T)

                palm_R_traj = PiecewiseQuaternionSlerp()
                palm_R_traj.Append(0.0, RotationMatrix(R_start))
                palm_R_traj.Append(1.0, RotationMatrix(R_end))

        return  t_sol, q_iiwa_sol, q_hand_sol,q_sol

    def ik_press_chord(self, q0, note_on_fingers):

        ik = InverseKinematics(self.plant, self.plant_context)

        T_const_fingertip_flat_lower = np.array([-self.key_size[0]*0.005,-self.key_size[1]*0.3,-self.key_size[2]*0.05])
        T_const_fingertip_flat_upper = np.array([+self.key_size[0]*0.005,+self.key_size[1]*0.1,+self.key_size[2]*0.2])

        T_const_fingertip_sharp_lower = np.array([-self.key_size[0]*0.005,-self.key_size[1]*0.35,-self.key_size[2]*0.05])
        T_const_fingertip_sharp_upper = np.array([+self.key_size[0]*0.005,+self.key_size[1]*0.3,+self.key_size[2]*0.2])

        T_center = self.note_dictionary[self.key_list[18]].pos
        T_const_release_lower = np.array([-1,-1, self.key_size[2]*1.2])
        T_const_release_upper = np.array([1,1,self.key_size[2]*10])

        T_average= np.array([0.0,0.0,0.0])       
        N_playing_key =0

        for idx, note in enumerate(note_on_fingers):
            if note != None and "none" in note:
                T_key = T_center
                T_lower = T_key + T_const_release_lower
                T_upper = T_key + T_const_release_upper
            else:
                key_type = self.note_dictionary[note].keytype

                if key_type =="flat":
                    T_key = self.note_dictionary[note].pos
                    T_lower = T_key + T_const_fingertip_flat_lower
                    T_upper = T_key + T_const_fingertip_flat_upper
                else :
                    T_key = self.note_dictionary[note].pos
                    T_lower = T_key + T_const_fingertip_sharp_lower
                    T_upper = T_key + T_const_fingertip_sharp_upper
                    
                T_average +=T_key
                N_playing_key+=1

            c = ik.AddPositionConstraint(
                self.finger_frames[idx], [0, 0, 0],self.plant.world_frame(),
                T_lower, T_upper)

            if "none" in note:
                1 # Do Nothing

            else:
                if key_type == "flat":
                    c = ik.AddAngleBetweenVectorsConstraint(
                        self.finger_frames[idx], [0, 0, 1],
                        self.plant.world_frame(), [0,0,-1],
                        0, np.pi/2*0.3
                    )
                    c = ik.AddAngleBetweenVectorsConstraint(
                        self.finger_frames[idx], [0, 0, 1],
                        self.plant.world_frame(), [1,0,0],
                        np.pi/2*0.97, np.pi/2*1.03
                    )

                    c = ik.AddAngleBetweenVectorsConstraint(
                        self.finger_frames[idx], [0, 0, 1],
                        self.plant.world_frame(), [1,0,0],
                        np.pi/2*0.9, np.pi/2*1.3
                    )
                else: #"sharp"
                    c = ik.AddAngleBetweenVectorsConstraint(
                        self.finger_frames[idx], [0, 0, 1],
                        self.plant.world_frame(), [0,0,-1],
                        0, np.pi/2*0.3
                    )
                    c = ik.AddAngleBetweenVectorsConstraint(
                        self.finger_frames[idx], [0, 0, 1],
                        self.plant.world_frame(), [1,0,0],
                        np.pi/2*0.7, np.pi/2*1.3
                    )

        if N_playing_key >0:
            T_average = T_average/N_playing_key
        else:
            T_average = T_center

        T_const_palm_lower = np.array([-1,-self.key_size[1]*5,self.key_size[2]*1])
        T_const_palm_upper = np.array([+1,self.key_size[1]*5,self.key_size[2]*10])


        T_const_palm_rest_lower = np.array([-1,-self.key_size[1]*5,self.key_size[2]*1.5])
        T_const_palm_rest_upper = np.array([+1,0.5,self.key_size[2]*10])

        if N_playing_key>0:

            T_palm_lower =T_average + T_const_palm_lower
            T_palm_upper =T_average + T_const_palm_upper

            if N_playing_key==1:
                c = ik.AddAngleBetweenVectorsConstraint(
                    self.palm_frame, [0, 0, 1],
                    self.plant.world_frame(), [0,1,0],
                    0, np.pi/2*0.6
                )
            else:
                c = ik.AddAngleBetweenVectorsConstraint(
                    self.palm_frame, [0, 0, 1],
                    self.plant.world_frame(), [0,1,0],
                    0, np.pi/2*0.8
                )

            c = ik.AddAngleBetweenVectorsConstraint(
                self.palm_frame, [1, 0, 0],
                self.plant.world_frame(), [0,0,-1],
                0, np.pi/2*0.5
            )               
            # c = ik.AddAngleBetweenVectorsConstraint(
            #     self.palm_frame, [1, 0, 0],
            #     self.plant.world_frame(), [-1,0,0],
            #     0, np.pi/2*0.5
            # )               
        else:
            T_palm_lower =T_average + T_const_palm_rest_lower
            T_palm_upper =T_average + T_const_palm_rest_upper
            
            # c = ik.AddAngleBetweenVectorsConstraint(
            #     self.palm_frame, [0, 0, 1],
            #     self.plant.world_frame(), [0,1,0],
            #     0, np.pi/2*0.4
            # )         

            # c = ik.AddAngleBetweenVectorsConstraint(
            #     self.palm_frame, [1, 0, 0],
            #     self.plant.world_frame(), [0,0,-1],
            #     0, np.pi/2*0.5
            # )                      

        c = ik.AddPositionConstraint(
            self.palm_frame , [0, 0, 0], self.plant.world_frame(),
            T_palm_lower, T_palm_upper)

        # T_const_hand_lower = np.array([-2,-2,self.key_size[2]*0.2])
        # T_const_hand_upper = np.array([+2,2,2])
        # T_hand_lower = T_center + T_const_hand_lower
        # T_hand_upper = T_center + T_const_hand_upper

        # c = ik.AddPositionConstraint(
        #     self.hand_frame, [0, 0, 0],self.plant.world_frame(),
        #     T_hand_lower, T_palm_upper)
                
        prog = ik.get_mutable_prog()
        q = ik.q()
        prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
        prog.SetInitialGuess(q, q0)
        result = Solve(ik.prog())

        q_iiwa = self.plant.GetPositions(self.plant_context, 
                    self.plant.GetModelInstanceByName("iiwa7"))
        q_hand = self.plant.GetPositions(self.plant_context, 
                    self.plant.GetModelInstanceByName("allegro_hand_right"))  

        return result.is_success(), q_iiwa, q_hand
    
    
    def music_sequence_to_trajectory(self, music_seq):
        # @ music_seq : list of Music_Note class
        _iiwa = self.plant.GetModelInstanceByName("iiwa7")
        _hand = self.plant.GetModelInstanceByName("allegro_hand_right")

        q0 = self.plant.GetPositions(self.plant_context)
        q0_iiwa = self.plant.GetPositions(self.plant_context,_iiwa)
        q0_hand = self.plant.GetPositions(self.plant_context,_hand)
        # q0_all = q0_init

        result_test, q_iiwa_init, q_hand_init,_ = self.ik_init(q0)
        q0 = self.plant.GetPositions(self.plant_context)
        q0_init = q0
        t_init = 0.1

        t = t_init
        c_hand_release = np.array([0.001,0.001,0.001,0.001])

        # q_iiwa_traj = PiecewisePolynomial.FirstOrderHold([0,t],np.vstack([q_iiwa_init, q_iiwa_init]).T)
        # q_hand_traj = PiecewisePolynomial.FirstOrderHold([0,t],np.vstack([q_hand_init, q_hand_init]).T)
        # q_all_traj = PiecewisePolynomial.FirstOrderHold([0,t],np.vstack([q0_init, q0_init]).T)
        
        # c_hand_traj =  PiecewisePolynomial.FirstOrderHold([0,t],np.vstack([c_hand_release, c_hand_release]).T)

        # self.plant.SetPositions(self.plant_context, _iiwa, q_iiwa)
        # self.plant.SetPositions(self.plant_context, _hand, q_hand)

        chord_none = "none"
        chord_result = []
        
        def notes_to_c_hand(notes):
            c_hand = np.array([0,0,0,0])
            for idx, note in enumerate(notes):
                if note != "none":
                    c_hand[idx] = 1.00

            return c_hand
                

        q0_past = self.plant.GetPositions(self.plant_context)
        t_transition_past = 3
        init_flag = False
        for idx, music_note in enumerate(music_seq):
            # q0 = self.plant.GetPositions(self.plant_context)
            # result_test, q_iiwa_hold, q_hand_hold, notes = self.ik_chord_to_finger(chord = music_note.name, octave = music_note.key_scale,q0 = q0)
            # self.plant.SetPositions(self.plant_context, _iiwa, q_iiwa_hold)
            # self.plant.SetPositions(self.pclant_context, _hand, q_hand_hold)


            if music_note.name =="none":

                    t_all = music_note.t_push+ music_note.t_release + music_note.t_hold + t_transition_past
                    # t+=t_all
                    
                    t_transition_past = t_all+ music_note.t_transition
                    # t_transition_past = music_note.t_transition

            else:
                # result_press, q_iiwa_push, q_hand_push, q_iiwa_release, q_hand_release, notes = self.ik_chord_to_finger(chord = music_note.name, octave = music_note.key_scale,q0 = q0)
                
                result_press, q_iiwa_prepress, q_hand_prepress, q_iiwa_press, q_hand_press, q_iiwa_release, q_hand_release, notes =  self.ik_chord_to_finger(chord = music_note.name, octave = music_note.key_scale,q0 = q0)
                c_hand_push = notes_to_c_hand(notes)

                self.plant.SetPositions(self.plant_context, _iiwa, q_iiwa_press)
                self.plant.SetPositions(self.plant_context, _hand, q_hand_press)
                q0_press = self.plant.GetPositions(self.plant_context)

                self.plant.SetPositions(self.plant_context, _iiwa, q_iiwa_prepress)
                self.plant.SetPositions(self.plant_context, _hand, q_hand_prepress)
                q0_prepress = self.plant.GetPositions(self.plant_context)

                self.plant.SetPositions(self.plant_context, _iiwa, q_iiwa_release)
                self.plant.SetPositions(self.plant_context, _hand, q_hand_release)
                q0_release = self.plant.GetPositions(self.plant_context)
                chord_result.append([music_note.name, str(music_note.key_scale), result_press])
                
                # q0_transition = (q0_release*0.3+q0_past*0.7)
                # result_transition, q_iiwa_transition, q_hand_transition = self.ik_transition(q0_transition)

                # self.plant.SetPositions(self.plant_context, _iiwa, q_iiwa_transition)
                # self.plant.SetPositions(self.plant_context, _hand, q_hand_transition)
                # q0_transition = self.plant.GetPositions(self.plant_context)

                # q0_transition = q0_release

                if init_flag==False:

                    q_iiwa_traj = PiecewisePolynomial.FirstOrderHold([0,0.5],np.vstack([q_iiwa_release, q_iiwa_release]).T)
                    q_hand_traj = PiecewisePolynomial.FirstOrderHold([0,0.5],np.vstack([q_hand_release, q_hand_release]).T)
                    q_all_traj = PiecewisePolynomial.FirstOrderHold([0,0.5],np.vstack([q0_release, q0_release]).T)
                    c_hand_traj =  PiecewisePolynomial.FirstOrderHold([0,0.5],np.vstack([c_hand_release, c_hand_release]).T)
                    self.q0_init = q0_release

                    t += t_transition_past
                    q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_release)  
                    q_hand_traj.AppendFirstOrderSegment(t, q_hand_release)
                    q_all_traj.AppendFirstOrderSegment(t, q0_release)
                    c_hand_traj.AppendFirstOrderSegment(t,c_hand_release)
                    init_flag =True
                else:


                    t_transition_list, q_arm_list, a_hand_list,q_sol_list = self.ik_transition_arm(q0_past, q0_prepress,2)
                    for idx, t_ratio in enumerate(t_transition_list):
                        t_now = t_transition_past*0.8*t_ratio
                        q_iiwa_now = q_arm_list[idx]
                        q_hand_now = a_hand_list[idx]
                        q_all_now = q_sol_list[idx]
                        
                        q_iiwa_traj.AppendFirstOrderSegment(t+t_now, q_iiwa_now)
                        q_hand_traj.AppendFirstOrderSegment(t+t_now, q_hand_now)
                        q_all_traj.AppendFirstOrderSegment(t+t_now, q_all_now)

 

                    t+=t_transition_past*0.8
                    q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_prepress)
                    q_hand_traj.AppendFirstOrderSegment(t, q_hand_prepress)
                    q_all_traj.AppendFirstOrderSegment(t, q0_prepress)
                    c_hand_traj.AppendFirstOrderSegment(t,c_hand_release)

                    t+=t_transition_past*0.2
                    q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_prepress)
                    q_hand_traj.AppendFirstOrderSegment(t, q_hand_prepress)
                    q_all_traj.AppendFirstOrderSegment(t, q0_prepress)
                    c_hand_traj.AppendFirstOrderSegment(t,c_hand_release)


                t += music_note.t_push
                q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_press)
                q_hand_traj.AppendFirstOrderSegment(t, q_hand_press)
                q_all_traj.AppendFirstOrderSegment(t, q0_press)
                c_hand_traj.AppendFirstOrderSegment(t,c_hand_push)

                t += music_note.t_hold
                q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_press)
                q_hand_traj.AppendFirstOrderSegment(t, q_hand_press)
                q_all_traj.AppendFirstOrderSegment(t, q0_press)
                c_hand_traj.AppendFirstOrderSegment(t,c_hand_push)

                t += music_note.t_release
                q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_release)  
                q_hand_traj.AppendFirstOrderSegment(t, q_hand_release)
                q_all_traj.AppendFirstOrderSegment(t, q0_release)
                c_hand_traj.AppendFirstOrderSegment(t,c_hand_release)
                
                t_transition_past = music_note.t_transition
                q0_past = q0_release
                q0 = q0_release
        t+=1
        q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_release)  
        q_hand_traj.AppendFirstOrderSegment(t, q_hand_release)
        q_all_traj.AppendFirstOrderSegment(t, q0_release)
        c_hand_traj.AppendFirstOrderSegment(t,c_hand_release)
                
        # q0_transition = (q0_release*0.5+q0_init*0.5)
        # result_transition, q_iiwa_transition, q_hand_transition = self.ik_transition(q0_transition)
        # self.plant.SetPositions(self.plant_context, _iiwa, q_iiwa_transition)
        # self.plant.SetPositions(self.plant_context, _hand, q_hand_transition)
        # q0_transition = self.plant.GetPositions(self.plant_context)

        # t+=1
        # q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_transition)   
        # q_hand_traj.AppendFirstOrderSegment(t, q_hand_transition)
        # q_all_traj.AppendFirstOrderSegment(t, q0_transition)
        # c_hand_traj.AppendFirstOrderSegment(t,c_hand_release)
        t+=4
        q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_init)   
        q_hand_traj.AppendFirstOrderSegment(t, q_hand_init)  
        q_all_traj.AppendFirstOrderSegment(t, q0_init)
        c_hand_traj.AppendFirstOrderSegment(t,c_hand_release)
        t+=1
        q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_init)   
        q_hand_traj.AppendFirstOrderSegment(t, q_hand_init)  
        q_all_traj.AppendFirstOrderSegment(t, q0_init)
        c_hand_traj.AppendFirstOrderSegment(t,c_hand_release)

        # t+=0.5
        # q_iiwa_traj.AppendFirstOrderSegment(t, q_iiwa_init)   
        # q_hand_traj.AppendFirstOrderSegment(t, q_hand_init)
        # q_all_traj.AppendFirstOrderSegment(t, q0_init)    
        # c_hand_traj.AppendFirstOrderSegment(t,c_hand_release)

        w_iiwa_traj = q_iiwa_traj.MakeDerivative()
        w_hand_traj = q_hand_traj.MakeDerivative()


        return q_iiwa_traj, q_hand_traj, q_all_traj, w_iiwa_traj, w_hand_traj, c_hand_traj, chord_result, self.q0_init

    def inverse_kinematics_test(self):

        q0 = self.plant.GetPositions(self.plant_context)

        key = []
        for key_name in (self.key_list):
            key.append(self.note_dictionary[key_name])

        self.test_cnt = 0
        def my_callback(context, pose):

            q0 = self.plant.GetPositions(self.plant_context)
            self.test_cnt +=1
            if self.test_cnt >=9:
                self.test_cnt = 0

            key_test =[]
            tt = self.test_cnt%3
            ts = int((self.test_cnt -tt)/3)
            if self.test_cnt%3 ==0:
                key_test.append(key[0+ts*12])
                key_test.append(key[0+ts*12])
                key_test.append(key[2+ts*12])
                key_test.append(key[4+ts*12])

            elif self.test_cnt%3 ==1:
                key_test.append(key[0+ts*12])
                key_test.append(key[0+ts*12])
                key_test.append(key[4+ts*12])
                key_test.append(key[7+ts*12])

            else:
                key_test.append(key[0+ts*12])
                key_test.append(key[8+ts*12])
                key_test.append(key[9+ts*12])
                key_test.append(key[10+ts*12])

            ik = InverseKinematics(self.plant, self.plant_context)

            # ik.AddPositionConstraint(
            #     self.finger_frames[0], [0, 0, 0], self.plant.world_frame(),
            #     pose.translation(), pose.translation())
            T_const_fingertip_lower = np.array([-self.key_size[0]*0.1,-self.key_size[1]*0.35,0])
            T_const_fingertip_upper = np.array([+self.key_size[0]*0.1,+self.key_size[1]*0.25,0])

            X_KR1 = RigidTransform(R=RotationMatrix.MakeYRotation(np.pi))
            X_KR2 = RigidTransform(R=RotationMatrix.MakeZRotation(-np.pi/2))
            R_FContact = (X_KR1.multiply(X_KR2)).rotation()

            T_average= np.array([0.0,0.0,0.0])
            N_finger_idx = 0
            c_list = []
            cc_list = []
         
            for idx in range(1,4):
                T_key = key_test[idx].pos
                T_average +=T_key
 
                T_lower = T_key + T_const_fingertip_lower
                T_upper = T_key + T_const_fingertip_upper
                
                c = ik.AddPositionConstraint(
                    self.finger_frames[idx], [0, 0, 0],self.plant.world_frame(),
                    T_lower, T_upper)
                c_list.append(c)

                cc = str(idx)+"Pos1"
                cc_list.append(cc)
                # ik.AddAngleBetweenVectorsConstraint(
                #     self.finger_frames[idx], [0, 0, -1],
                #     self.plant.world_frame(), [1,0,0],
                #     np.pi/2*0.9, np.pi/2*1.1
                # )
                # ik.AddAngleBetweenVectorsConstraint(
                #     self.finger_frames[idx], [0, 0, -1],
                #     self.plant.world_frame(), [0,1,0],
                #     np.pi/2*0.8, np.pi/2*1.2
                # )
                c = ik.AddAngleBetweenVectorsConstraint(
                    self.finger_frames[idx], [0, 0, 1],
                    self.plant.world_frame(), [0,0,-1],
                    0, np.pi/2*0.3
                )

                cc = str(idx)+"angle1"
                cc_list.append(cc)

                c_list.append(c)
                c = ik.AddAngleBetweenVectorsConstraint(
                    self.finger_frames[idx], [1, 0, 1],
                    self.plant.world_frame(), [0,-1,0],
                    0, np.pi/2*0.4
                )
                cc = str(idx)+"angle2"
                cc_list.append(cc)

                c_list.append(c)
                # c = ik.AddAngleBetweenVectorsConstraint(
                #     self.finger_frames[idx], [0, 1, 1],
                #     self.plant.world_frame(), [-1,0,0],
                #     0, np.pi/2*0.25
                # )
                # cc = str(idx)+"angle3"
                # cc_list.append(cc)
                
                c_list.append(c)
                N_finger_idx +=1

            T_average = T_average/N_finger_idx
            T_const_palm_lower = np.array([-0.5,-self.key_size[1],self.key_size[2]*0.05])
            T_const_palm_upper = np.array([+0.5,self.key_size[1],self.key_size[2]*3])
            
            T_palm_lower =T_average + T_const_palm_lower
            T_palm_upper =T_average + T_const_palm_upper
            c = ik.AddPositionConstraint(
                self.palm_frame , [0, 0, 0], self.plant.world_frame(),
                T_palm_lower, T_palm_upper)

            c = ik.AddPositionConstraint(
                self.finger_frames[0], [0, 0, 0],self.plant.world_frame(),
                T_palm_lower, T_palm_upper)   

            c_list.append(c)
            cc = "palm_pos1"
            cc_list.append(cc)
            ik.AddAngleBetweenVectorsConstraint(
                self.palm_frame, [0, 0, 1],
                self.plant.world_frame(), [0,1,0],
                0, np.pi/2*0.5
            )

            c_list.append(c)
            cc = "palm_angle1"
            cc_list.append(cc)
  
            prog = ik.get_mutable_prog()
            q = ik.q()
            prog.AddQuadraticErrorCost(np.identity(len(q)), q0, q)
            prog.SetInitialGuess(q, q0)
            result = Solve(ik.prog())
            #print(c_list)
            #print(cc_list)
            # print(result.GetSolution())
            q_iiwa = self.plant.GetPositions(self.plant_context, 
                        self.plant.GetModelInstanceByName("iiwa7"))
            print(q_iiwa)
            #print(result.GetInfeasibleConstraints(prog,0.01))
            if self.test_cnt==8:
                clear_output(wait=True)

            #print((self.test_cnt))
            if result.is_success():
                print("IK success_{}".format(self.test_cnt))
            else:
                print("IK failure_{}".format(self.test_cnt))

        sliders = MeshcatPoseSliders(get_meshcat())
        
        # sliders.SetPose(self.plant.EvalBodyPoseInWorld(
        #     self.plant_context, self.plant.GetBodyByName("link_7",self.model_hand)))

        # set the initial z lower, to make the interaction interesting.
        sliders.SetXyz([0.4, 0.2, 0.65])
        sliders.Run(self.visualizer, self.context, my_callback)
        get_meshcat().DeleteAddedControls()
        get_meshcat().Delete()

        return

    def ik_chord_to_finger(self, chord, octave,q0):

        notes_key = generate_note_from_chord(chord, octave)
        N_sol, fidx, cost = self.find_finger_correspondence(notes_key)
        cost_idx= np.argsort(cost[:,0])

        result = False

        q0 = self.plant.GetPositions(self.plant_context)

        for i in range(N_sol):
            notes = ["none","none","none","none"]
            n_idx = 0
            
            for ff in fidx[cost_idx[i]] :

                notes[int(ff)] = notes_key[n_idx]

                n_idx+=1
            
            result_prepress, q_iiwa_prepress, q_hand_prepress = self.ik_prepress_chord(q0, notes)
            q0_prepress  = self.plant.GetPositions(self.plant_context)

            result_press, q_iiwa_press, q_hand_press = self.ik_press_chord(q0_prepress, notes)
            q0_press  = self.plant.GetPositions(self.plant_context)

            result_release, q_iiwa_release, q_hand_release = self.ik_prepress_chord(q0_press, notes)

            result =result_prepress and result_press and result_release

            if result:
                break

        if result == False:
            notes = ["none","none","none","none"]
            i=0
            n_idx=0
            for ff in fidx[cost_idx[i]] :
                notes[int(ff)] = notes_key[n_idx]
                n_idx+=1


            result_prepress, q_iiwa_prepress, q_hand_prepress = self.ik_prepress_chord(q0, notes)
            q0_prepress  = self.plant.GetPositions(self.plant_context)

            result_press, q_iiwa_press, q_hand_press = self.ik_press_chord(q0_prepress, notes)
            q0_press  = self.plant.GetPositions(self.plant_context)

            result_release, q_iiwa_release, q_hand_release = self.ik_prepress_chord(q0_press, notes)

            result =result_prepress and result_press and result_release

        # return result_test, q_iiwa, q_hand, notes
        return result, q_iiwa_prepress, q_hand_prepress, q_iiwa_press, q_hand_press, q_iiwa_release, q_hand_release, notes
        
    def inverse_kinematics_test2(self):

        q0 = self.plant.GetPositions(self.plant_context)

        key = []
        for key_name in (self.key_list):
            key.append(self.note_dictionary[key_name])

        self.test_cnt = 0

        def my_callback(context, pose):

            q0 = self.plant.GetPositions(self.plant_context)
            N_chord = len(chord_list)
            self.test_cnt +=1
            if self.test_cnt >=N_chord*3:
                clear_output(wait=True)
                self.test_cnt = 0

            ii = self.test_cnt%N_chord
            ss = int(self.test_cnt/N_chord)+1
            # c_test = "CM"
            result_test,_,_, q_iiwa, q_hand, _, _,notes = self.ik_chord_to_finger(chord = chord_list[ii], octave = ss,q0 = q0)

            # if(self.test_cnt %3 ==0):
            #     q0 = self.plant.GetPositions(self.plant_context)
            #     result_test, q_iiwa, q_hand = self.ik_init(q0=q0)

            _iiwa = self.plant.GetModelInstanceByName("iiwa7")
            _hand = self.plant.GetModelInstanceByName("allegro_hand_right")
            self.plant.SetPositions(self.plant_context, _iiwa, q_iiwa)
            self.plant.SetPositions(self.plant_context, _hand, q_hand)

            
            if result_test:
                print("IK success_{}, ocatve:{}, chord:{}, notes:{}".format(self.test_cnt,ss,chord_list[ii],notes))
            else:
                print("IK failure_{}, ocatve:{}, chord:{}, notes:{}".format(self.test_cnt,ss,chord_list[ii],notes))


        sliders = MeshcatPoseSliders(get_meshcat())
        
        # sliders.SetPose(self.plant.EvalBodyPoseInWq0_initorld(
        #     self.plant_context, self.plant.GetBodyByName("link_7",self.model_hand)))

        # # set the initial z lower, to make the interaction interesting.
        # sliders.SetXyz([0.4, 0.2, 0.65])
        sliders.Run(self.visualizer, self.context, my_callback)
        get_meshcat().DeleteAddedControls()
        get_meshcat().Delete()

        return
