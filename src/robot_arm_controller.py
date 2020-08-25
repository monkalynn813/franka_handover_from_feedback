#!/usr/bin/env python
import rospy
import moveit_commander
from franka_interface import ArmInterface, GripperInterface
from franka_moveit import PandaMoveGroupInterface 
from math import pi, sqrt
import numpy as np
import tf.transformations as tr
import tf_conversions.posemath as pm
from trajectory_generator import msg_to_se3, se3_to_msg

#import rosmsg needed:
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String, Bool



class Impedance_control():
    def __init__(self,ptx=150,pty=150,ptz=150,pr=10,mode='static'):
        #set stiffness:
        self.P = np.zeros([6,6])
        np.fill_diagonal(self.P,[ptx,pty,ptz,pr,pr,pr])
        #set damping: critical damp
        self.D= np.zeros([6,6])
        np.fill_diagonal(self.D,2*np.sqrt(np.diag(self.P)))
        #TODO: set inertia
        

        self.mode=mode

        self.joint_names=['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
        self.limb = ArmInterface()

        if mode =='static':
            self.panda_moveit_wrap = PandaMoveGroupInterface()
            self.arm_group=self.panda_moveit_wrap._arm_group
        #publisher:
        # self.publisher_name = rospy.Publisher('topic_name',String)

        #subscriber:
        # self.subscriber_name = rospy.Subscriber('topic_name',String, self.subscriber_callback)

        rate=rospy.Rate(10) 
    
    def get_reference_pose(self,target_pose=None):
        #target_pose= geometry_msgs/Pose
        if self.mode=='static':
            self.go_static_test_pose()
            rospy.sleep(2.0)
            rospy.loginfo('---Moved to static test pose----')
            self.ref_position=self.limb.endpoint_pose()['position']
            self.ref_orientation=self.limb.endpoint_pose()['orientation']
        elif self.mode=='dynamic':
            self.ref_position=target_pose.position
            self.ref_orientation=target_pose.orientation

        
    def go_static_test_pose(self):
        rospy.loginfo("--moving to static test pose---")

        T_standby=np.array([[-0.1815108 , -0.98090692,  0.06982443, -0.15],
                            [ 0.00259123,  0.07052656,  0.99750654,  0.5],
                            [-0.98338554,  0.18123914, -0.01025958,  0.5],
                            [ 0.        ,  0.        ,  0.        ,  1. ]])

        target_pose= se3_to_msg(T_standby)
        self.arm_group.set_pose_target(target_pose)
        self.arm_group.go(wait=True) 
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
    
    def control_loop(self,ref_position,ref_orientation):
        #TODO: get coriolis
        #get Jacobian
        J = self.limb.zero_jacobian()
        self.cur_poistion=self.limb.endpoint_pose()['position']
        self.cur_orientation=self.limb.endpoint_pose()['orientation']

        #compute deviation
        error=[]
        error.append(self.ref_position.x-self.cur_poistion.x)
        error.append(self.ref_position.y-self.cur_poistion.y)
        error.append(self.ref_position.z-self.cur_poistion.z)
        error.append(self.ref_orientation.x-self.cur_orientation.x)
        error.append(self.ref_orientation.y-self.cur_orientation.y)
        error.append(self.ref_orientation.z-self.cur_orientation.z)
        error=np.array(error).reshape(6,1)

        error_dot=[]
        #get joint velocities
        cur_dq=self.limb.joint_velocities()
        for name in self.joint_names:
            error_dot.append(cur_dq[name])
        error_dot=np.array(error_dot).reshape(7,1)
        d_error=np.dot(J,error_dot)

        wrench=np.dot(self.P,error) + np.dot(self.D, d_error)
        tau_task=np.dot(J.T,wrench) 
        #TODO: add coriolis
        tau_command=tau_task

        self.limb.set_joint_torques(dict(zip(self.joint_names,tau_command)))

    # def publish_function(self,arg):
        
    #     PLACEHOLDER
        
    #     self.publisher_name.publish(data)

    # def subscriber_callback(self,data):

    #     PLACEHOLDER

def main():
    rospy.init_node("robot_arm_controller")
    try:
        project=ProjectName()  #if program run from init, otherwise put starting function
        
    except rospy.ROSInterruptException:pass
        

    rospy.spin()
if __name__ == '__main__':
	main()
    