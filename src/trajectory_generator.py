#!/usr/bin/env python
import rospy
from franka_interface import ArmInterface
from math import pi, sqrt
import numpy as np
import tf.transformations as tr
import tf_conversions.posemath as pm
from modern_robotics import IKinSpace

#import rosmsg needed:
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from std_msgs.msg import Float64MultiArray


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
    def __init__(self,dmin=0,dmax=0.1,freq=5):
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
            self.T_rh=np.dot(np.linalg.inv(self.T_sr),T_sh)
            #call IK solution to pub target joint positions
            dis=self.get_current_distance(self.T_rh)
            if dis<=self.dmax and dis>=self.dmin:
                self.traj_to_target_pose(self.T_rh)
    def compute_hand_speed(self,hand_pose):
        dis=self.get_current_distance(self.T_rh)
        hand_pose_array=[]
        time=0
        if dis <=1.0 and dis>=self.dmax+0.1:
            hand_pose_array.append(hand_pose)
            time+=self.freq
        
        
                   
    def traj_to_target_pose(self,hand_pose):   
        #compute distance:
        cur_joint_pos=[]
        #get current joint position
        cur_q=self.limb.joint_angles()
        for name in self.joint_names:
            cur_joint_pos.append(cur_q[name]) 
        hand_position=hand_pose[:3,3]
        #set target ee pose
        #TODO more detail needed for robot hand pose relative to human hand
        target_pose=np.array([[ 0,            0,          1,  hand_position[0]-0.05],
                              [ -1/sqrt(2),  -1/sqrt(2),  0,  hand_position[1]],
                              [ 1/sqrt(2),    -1/sqrt(2), 0,  hand_position[2]],
                              [ 0,  0,  0, 1]])

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
    