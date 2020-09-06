#!/usr/bin/env python
import rospy
from franka_interface import ArmInterface
from math import pi, sqrt
import numpy as np
import tf.transformations as tr
import tf_conversions.posemath as pm
from modern_robotics import IKinSpace
import time

#import rosmsg needed:
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from std_msgs.msg import Float64MultiArray
from handover_controller.srv import enable_gripper_controller


def msg_to_se3(msg):
    """
    convert geometry_msgs/Pose to a transformation matrix SE(3)
    """
    return pm.toMatrix(pm.fromMsg(msg))

def se3_to_msg(T):
    """
    convert a transformation matrix SE(3) to geometry_msgs/Pose
    """
    return pm.toMsg(pm.fromMatrix(T))

class Trajectory_Generator():
    def __init__(self,dmin=0,dmax=0.1,freq=10):
        """
        arguments:
            dmin: the distance indicates robot to move along with virtual human hand position
            dmax: the distance indicates robot to start move along with human hand position
            T_ch: the transformation matrix of human hand w.r.t. depth camera frame
        """
        self.joint_names=['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

        self.dmax=dmax
        self.dmin=dmin

        self.limb = ArmInterface()
        
        self.T_sr= np.zeros([4,4])
        self.T_r_r=np.array([[0,1,0,0], #actual frobot frame wrt robot frame in optitrack sys
                             [0,0,1,0],
                             [1,0,0,0],
                             [0,0,0,1]])
        #screw axes (zero jacobian of franka)
        l1=0.3330
        l2=0.3160
        l3=0.3840
        l4=0.0880
        l5=0.1070

        self.Slist = np.array([[0,  0,  1,  0,  0,  0],
                                [0, 1,  0,  -l1, 0, 0],
                                [0, 0,  1,  0,  0,  0],
                                [0, -1, 0,  l1+l2,0,-l4],
                                [0, 0,  1,  0,   0,  0],
                                [0, -1, 0,  l1+l2+l3,0,0],
                                [0, 0,  -1, 0,   l4,  0]]).T
        
        self.M = np.array([[1, 0, 0, l4],
                           [0, -1, 0, 0],
                           [0, 0, -1, l1+l2+l3-l5],
                           [0, 0, 0, 1]])

        #publisher:
        self.traj_publisher = rospy.Publisher('robot_trajectory',Float64MultiArray,queue_size=10)
        self.gripper_controller_switch=rospy.ServiceProxy('grippper_controller_switch',enable_gripper_controller)
        #subscriber:
        self.freq=freq
        self.rate=rospy.Rate(freq)
        while not rospy.is_shutdown():
            robot_pose_= rospy.wait_for_message('vrpn_client_node/robot_pose/pose',PoseStamped)
            hand_pose_ = rospy.wait_for_message('vrpn_client_node/hand_pose/pose',PoseStamped)
            self.get_robot_pose(robot_pose_.pose)
            self.compute_target_pose(hand_pose_.pose)
            
            self.rate.sleep()
            
        
    def get_robot_pose(self,robot_pose):        
        T_sr_=msg_to_se3(robot_pose)
        self.T_sr=np.dot(T_sr_,self.T_r_r)
        
    def compute_target_pose(self,hand_pose):
        if self.T_sr.any() != np.zeros([4,4]).any():
            T_sh=msg_to_se3(hand_pose)
            T_rh=np.dot(np.linalg.inv(self.T_sr),T_sh)
            dis=self.get_current_distance(T_rh)
            #get speed of human hand within certain distance measurement and assume it to be constant
            speed=self.compute_hand_speed(dis)
            if speed:
                rospy.loginfo("Hand speed is %0.3f m/s" %speed)
                #TODO send speed to policy
            #call IK solution to pub target joint positions
            if dis<=self.dmax:
                gripper_status=self.gripper_controller_switch(True)
                if dis>=self.dmin:
                    self.traj_to_target_pose(T_rh)
                # TODO elif dis<dmin: 
               
        
    def compute_hand_speed(self,dis):
        epsilon=0.01
        measure_pt1=1.5 #m
        measure_pt2=0.8 #m
        speed=None
        if dis>= measure_pt1-epsilon and dis<=measure_pt1+epsilon:
            self.measure_timestp1=time.time()
        if dis>=measure_pt2-epsilon and dis<=measure_pt2+epsilon and 'self.meaure_timestp1' in locals() :
            measure_timestp2=time.time()
            speed=(measure_pt1-measure_pt2)/(measure_timestp2-self.measure_timestp1)
            del self.measure_timestp1
            del measure_timestp2
        return speed

        
    def traj_to_target_pose(self,hand_pose):   
        #compute distance:
        cur_joint_pos=[]
        #get current joint position
        cur_q=self.limb.joint_angles()
        for name in self.joint_names:
            cur_joint_pos.append(cur_q[name]) 
        hand_position=hand_pose[:3,3]
        x=hand_position[0]
        y=hand_position[1]
        z=hand_position[2]
        #set target ee pose
        #normalize hand position x,y for z screw axis of target pose
        zscrew_x=x/np.linalg.norm((x,y))
        zscrew_y=y/np.linalg.norm((x,y))
        zscrew=np.array([zscrew_x,zscrew_y,0])
        #make sure gripper is always flat
        xscrew=np.array([0,0,1]) 
        yscrew=np.cross(zscrew,xscrew)
        theta=pi/4
        Rrot=np.array([[np.cos(theta), -np.sin(theta), 0],
                       [np.sin(theta), np.cos(theta),  0],
                       [0,              0,             1]])
        #matain certain distance between hand and gripper
        l=0.05
        
        R=np.array([[ xscrew[0],    yscrew[0],  zscrew_x],
                    [ xscrew[1],    yscrew[1],  zscrew_y],
                    [ xscrew[2],    yscrew[2],  0,      ]])
        #rotate along z axis of the gripper frame by 45 degree
        target_rotM=np.dot(R,Rrot)

        target_pose=np.append(target_rotM,[[0,0,0]],axis=0)
        target_pose=np.append(target_pose,np.array([[x-l*zscrew_x,y-l*zscrew_y,z,1]]).reshape(4,1),axis=1)

        joint_target,success=IKinSpace(self.Slist,self.M,target_pose,np.array(cur_joint_pos),0.01,0.005)
        if success:
            message=Float64MultiArray()
            message.data=joint_target
            self.traj_publisher.publish(message)
        else:
            rospy.logwarn('Cannot solve IK frm current position')

    def get_current_distance(self,hand_pose):
        #compute current distance between human hand and end-effector
        current_T_be_position = self.limb.endpoint_pose()['position']
        d_square=0
        for i in range(3):
            d_square+=(current_T_be_position[i]-hand_pose[i,-1])**2
        return sqrt(d_square)

def main():
    rospy.init_node("trajectory_generator")
    try:
        trajectory_generator=Trajectory_Generator()  #if program run from init, otherwise put starting function
    
    except rospy.ROSInterruptException:pass
        
if __name__ == '__main__':
	main()
    