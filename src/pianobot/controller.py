"""Drake LeafSystem controllers for the robotic hand."""

import numpy as np
from pydrake.all import (
    BasicVector,
    JacobianWrtVariable,
    LeafSystem,
)


class MyControllerSystem(LeafSystem):
    """Wrapper system for Dynamic torque input
    """ 

    def __init__(self, plant):#, diffik_fun):
        LeafSystem.__init__(self)
        self._plant = plant
        self._plant_context = plant.CreateDefaultContext()
        # self._G = plant.GetBodyByName("body").body_frame()
        self._W = plant.world_frame()
        self._iiwa = plant.GetModelInstanceByName("iiwa7")
        self._hand = plant.GetModelInstanceByName("allegro_hand_right")

        finger_name_list = ["link_15", "link_3", "link_7","link_11"]
        self._F0 = plant.GetFrameByName("frame_fingercontact_"+finger_name_list[0])
        self._F1 = plant.GetFrameByName("frame_fingercontact_"+finger_name_list[1])
        self._F2 = plant.GetFrameByName("frame_fingercontact_"+finger_name_list[2])
        self._F3 = plant.GetFrameByName("frame_fingercontact_"+finger_name_list[3])

        self._BF0 = plant.GetBodyByName(finger_name_list[0]).body_frame()
        self._BF1 = plant.GetBodyByName(finger_name_list[1]).body_frame()
        self._BF2 = plant.GetBodyByName(finger_name_list[2]).body_frame()
        self._BF3 = plant.GetBodyByName(finger_name_list[3]).body_frame()
        
        self._P = plant.GetFrameByName("frame_palm")
        
        # define indices of the robot link
        iiwa_indices =np.array([0,61,62,63,64,65,66])
        hand_indices =np.array([67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82])
        self.robot_indices = np.hstack([iiwa_indices,hand_indices] )
        
        # define ports
        self.iiwa7_state_port = self.DeclareVectorInputPort("iiwa7_state", BasicVector(7*2))
        self.hand_state_port = self.DeclareVectorInputPort("hand_state", BasicVector(16*2))

        self.iiwa7_des_pos_port = self.DeclareVectorInputPort("iiwa7_des_pos", BasicVector(7))
        self.hand_des_pos_port = self.DeclareVectorInputPort("hand_des_pos", BasicVector(16))

        self.iiwa7_des_vel_port = self.DeclareVectorInputPort("iiwa7_des_vel", BasicVector(7))
        self.hand_des_vel_port = self.DeclareVectorInputPort("hand_des_vel", BasicVector(16))

        self.finger_des_contact_port = self.DeclareVectorInputPort("finger_des_contact", BasicVector(4))


        # self.hand_contact_force_port = self.DeclareVectorInputPort("hand_contact_forces",16)
        # self.contact_result_port = self.DeclareAbstractInputPort("contact_result", AbstractValue.Make(ContactResults()))


        self.DeclareVectorOutputPort("iiwa7_torque", BasicVector(7),self.calc_iiwa7_torque)
        self.DeclareVectorOutputPort("hand_torque", BasicVector(16),self.calc_hand_torque)
        self.cnt=0
        
        self.u_iiwa =np.zeros((7,1))
        self.u_hand =np.zeros((16,1))
        
    def calc_J(self,q_iiwa,q_hand):
        self._plant.SetPositions(self._plant_context, self._iiwa, q_iiwa)
        self._plant.SetPositions(self._plant_context, self._hand, q_hand)

        J_F0 = self._plant.CalcJacobianSpatialVelocity(
            self._plant_context, JacobianWrtVariable.kV, 
            self._BF0, [0,0,0], self._W, self._W)
        J_F0 = J_F0[0:3,self.robot_indices]

        J_F1 = self._plant.CalcJacobianSpatialVelocity(
            self._plant_context, JacobianWrtVariable.kV, 
            self._BF1, [0,0,0], self._W, self._W)
        J_F1 = J_F1[0:3,self.robot_indices]

        J_F2 = self._plant.CalcJacobianSpatialVelocity(
            self._plant_context, JacobianWrtVariable.kV, 
            self._BF2, [0,0,0], self._W, self._W)
        J_F2 = J_F2[0:3,self.robot_indices]

        J_F3 = self._plant.CalcJacobianSpatialVelocity(
            self._plant_context, JacobianWrtVariable.kV, 
            self._BF3, [0,0,0], self._W, self._W)
        J_F3 = J_F3[0:3,self.robot_indices]


        return J_F0, J_F1, J_F2, J_F3#, J_P

    def calc_pose(self, q_iiwa,q_hand):
        self._plant.SetPositions(self._plant_context, self._iiwa, q_iiwa)
        self._plant.SetPositions(self._plant_context, self._hand, q_hand)

        pose_F0 = self.get_pose(self._F0)
        pose_F1 = self.get_pose(self._F1)
        pose_F2 = self.get_pose(self._F2)
        pose_F3 = self.get_pose(self._F3)
        #pose_P = self.get_pose(self._P)
        
        return pose_F0, pose_F1, pose_F2, pose_F3#, pose_P

    def get_pose(self, frame):
        return self._plant.CalcRelativeTransform(
                            self._plant_context,
                            frame_A=self._plant.GetFrameByName("frame_top_ground"),
                            frame_B=frame)


    def main_controller(self, context):
        state_iiwa7 =  self.iiwa7_state_port.Eval(context)
        des_pos_iiwa7 =  self.iiwa7_des_pos_port.Eval(context)
        des_vel_iiwa7 =  self.iiwa7_des_vel_port.Eval(context)

        q_iiwa7 =np.zeros((7,1))
        dq_iiwa7 = np.zeros((7,1))

        q_iiwa7[:,0] = state_iiwa7[:7]
        dq_iiwa7[:,0] = state_iiwa7[7:]

        q_iiwa7_des = np.zeros((7,1))
        q_iiwa7_des[:,0] = des_pos_iiwa7
        dq_iiwa7_des = np.zeros((7,1))
        dq_iiwa7_des[:,0] = des_vel_iiwa7
        
        state_hand =  self.hand_state_port.Eval(context)
        des_pos_hand =  self.hand_des_pos_port.Eval(context)
        des_vel_hand =  self.hand_des_vel_port.Eval(context)
        
        q_hand =np.zeros((16,1))
        dq_hand = np.zeros((16,1))

        q_hand[:,0] = state_hand[:16]
        dq_hand[:,0] = state_hand[16:]

        q_hand_des = np.zeros((16,1))
        q_hand_des[:,0] = des_pos_hand
        dq_hand_des = np.zeros((16,1))
        dq_hand_des[:,0] = des_vel_hand

        des_contact_finger = self.finger_des_contact_port.Eval(context)
        
        K_iiwa = 2*np.array([[1000, 800, 800, 700,500,400,300]]).T
        D_iiwa = K_iiwa*0.22
        u_iiwa_pd = np.multiply(K_iiwa,(q_iiwa7_des-q_iiwa7))+np.multiply(D_iiwa,dq_iiwa7_des-dq_iiwa7) 
        self.u_iiwa = u_iiwa_pd
             
        # define baseline joint gains
        K_hand = 1.8*np.array([[2,2,2,2, 1,1,1,1, 1,1,1,1, 0.5,0.5,0.5,0.5]]).T
        
        # apply contact mode gains
        K_hand_contact = K_hand*10
        K_hand_contact[0,0] *= des_contact_finger[1]
        K_hand_contact[4,0] *= des_contact_finger[1]
        K_hand_contact[8,0] *= des_contact_finger[1]
        K_hand_contact[12,0] *= des_contact_finger[1]

        K_hand_contact[1,0] *= des_contact_finger[0]
        K_hand_contact[5,0] *= des_contact_finger[0]
        K_hand_contact[9,0] *= des_contact_finger[0]
        K_hand_contact[13,0] *= des_contact_finger[0]

        K_hand_contact[2,0] *= des_contact_finger[2]
        K_hand_contact[6,0] *= des_contact_finger[2]
        K_hand_contact[10,0] *= des_contact_finger[2]
        K_hand_contact[14,0] *= des_contact_finger[2]

        K_hand_contact[3,0] *= des_contact_finger[3]
        K_hand_contact[7,0] *= des_contact_finger[3]
        K_hand_contact[11,0] *= des_contact_finger[3]
        K_hand_contact[15,0] *= des_contact_finger[3]

        K_hand += K_hand_contact
        
        D_hand = K_hand*0.16
        
        u_hand_pd = np.multiply(K_hand,(q_hand_des-q_hand))+np.multiply(D_hand,dq_hand_des-dq_hand)


        u_hand = u_hand_pd

        # Re-mapping the gains. 
        # This is becasue joint index and actuator index is not aligned in the model.

        self.u_hand = u_hand*0
        self.u_hand[0,0] = u_hand[0,0]
        self.u_hand[4,0] = u_hand[1,0]
        self.u_hand[8,0] = u_hand[2,0]
        self.u_hand[12,0] = u_hand[3,0]

        self.u_hand[1,0] = u_hand[4,0]
        self.u_hand[5,0] = u_hand[5,0]
        self.u_hand[9,0] = u_hand[6,0]
        self.u_hand[13,0] = u_hand[7,0]

        self.u_hand[2,0] = u_hand[8,0]
        self.u_hand[6,0] = u_hand[9,0]
        self.u_hand[10,0] = u_hand[10,0]
        self.u_hand[14,0] = u_hand[11,0]

        self.u_hand[3,0] = u_hand[12,0]
        self.u_hand[7,0] = u_hand[13,0]
        self.u_hand[11,0] = u_hand[14,0]
        self.u_hand[15,0] = u_hand[15,0]

    def calc_iiwa7_torque(self, context, output):
        self.main_controller(context)
 
        output.SetFromVector(self.u_iiwa)

    def calc_hand_torque(self, context, output):
        self.main_controller(context)
        output.SetFromVector(self.u_hand)
