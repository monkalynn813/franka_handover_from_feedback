#!/usr/bin/env python
import rospy
import moveit_commander
from franka_interface import ArmInterface, GripperInterface
from franka_moveit import PandaMoveGroupInterface 
from math import pi, sqrt
import numpy as np
import tf.transformations as tr
import tf_conversions.posemath as pm

#import rosmsg needed:
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from  moveit_msgs.msg import RobotTrajectory
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String, Bool


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
        self.dmax=dmax
        self.dmin=dmin

        self.limb = ArmInterface()
        self.panda_moveit_wrap = PandaMoveGroupInterface()

        self.arm_group=self.panda_moveit_wrap._arm_group
        self.gripper_group= self. panda_moveit_wrap._gripper_group

        self.T_bc= np.array([[1,0,0,1],
                            [0,1,0,0.5],
                            [0,0,1,0],
                            [0,0,0,1]])

        self.T_bh= np.zeros([4,4])
        #publisher:
        self.traj_publisher = rospy.Publisher('robot_trajectory',RobotTrajectory,queue_size=10)

        # self.go_standby_position()
        # rospy.sleep(2.0)

        #subscriber:
        self.hand_pose_subscriber = rospy.Subscriber('virtual_hand_pose',Pose, self.traj_to_target_pose)
        
          
    
    def go_standby_position(self):
        rospy.loginfo("--moving to standby position---")
        ##########for virtual hand position test purpose#############
        radius= 0.15
        circle_center=[0,0.65] #(x,z)
        x=circle_center[0]-radius
        z= sqrt(radius**2-(round(x-circle_center[0],3))**2)+circle_center[1]
        T_standby=np.array([[-0.1815108 , -0.98090692,  0.06982443, x],
                            [ 0.00259123,  0.07052656,  0.99750654,  0.5],
                            [-0.98338554,  0.18123914, -0.01025958,  z],
                            [ 0.        ,  0.        ,  0.        ,  1. ]])
             # ############################################################
        target_pose= se3_to_msg(T_standby)
        self.arm_group.set_pose_target(target_pose)
        self.arm_group.go(wait=True) 
        # self.arm_group.stop()
        self.arm_group.clear_pose_targets()

    
    def test_joint_pos_command(self):
        joint_goal = self.arm_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -3*pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/2
        joint_goal[6] = pi/4

        self.panda_moveit_wrap.go_to_joint_positions(joint_goal)
    
    def traj_to_target_pose(self,hand_pose):                 
        #get current ee pose
        cur_pose= se3_to_msg(self.limb.car_pose_trans_mat)

        #compute robot target pose
        #TODO more detail needed to compute robot target pose from human pose
        target_pose=hand_pose
        
        #add waypoints in this trajectory
        waypoints=[]
        #append only start/current and end/target pose for now
        waypoints.append(cur_pose)
        waypoints.append(target_pose)

        #compute cartesian plan
        traj_plan=RobotTrajectory()
        (traj_plan,fraction)=self.arm_group.compute_cartesian_path(waypoints, 0.02, 0.0)
        self.traj_publisher.publish(traj_plan)



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
    