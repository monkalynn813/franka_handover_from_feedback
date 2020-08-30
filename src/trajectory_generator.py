#!/usr/bin/env python
import rospy
from franka_interface import ArmInterface, GripperInterface
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
    def __init__(self,dmin=None,dmax=None):
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
        
        self.T_bc= np.array([[1,0,0,1],
                            [0,1,0,0.5],
                            [0,0,1,0],
                            [0,0,0,1]])

        self.T_bh= np.zeros([4,4])

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
        self.hand_pose_subscriber = rospy.Subscriber('virtual_hand_pose',Pose, self.traj_to_target_pose)
        
          
   
    def traj_to_target_pose(self,hand_pose):                 
        cur_joint_pos=[]
        #get current joint position
        cur_q=self.limb.joint_angles()
        for name in self.joint_names:
            cur_joint_pos.append(cur_q[name]) 
        
        #set target ee pose
        #TODO more detail needed for robot hand pose relative to human hand
        target_pose=msg_to_se3(hand_pose)

        joint_target,success=IKinSpace(self.Slist,self.M,target_pose,np.array(cur_joint_pos),0.01,0.005)
        if success:
            message=Float64MultiArray()
            message.data=joint_target
            self.traj_publisher.publish(message)
        else:
            rospy.logwarn('Cannot solve IK frm current position')
    


    def get_current_distance(self):
        #compute current distance between human hand and end-effector
        current_T_be_position = self.limb.endpoint_pose()['position']

        d_square=0
        for i in range(3):
            d_square+=(current_T_be_position[i]-self.T_bh[i,-1])**2
        return sqrt(d_square)

def main():
    rospy.init_node("trajectory_generator")
    try:
        trajectory_generator=Trajectory_Generator()  #if program run from init, otherwise put starting function
    
    except rospy.ROSInterruptException:pass
        

    rospy.spin()
if __name__ == '__main__':
	main()
    