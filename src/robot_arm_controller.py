#!/usr/bin/env python
import rospy
import moveit_commander
from franka_interface import ArmInterface, GripperInterface
from franka_moveit import PandaMoveGroupInterface 
from franka_tools import CollisionBehaviourInterface
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
        
        self.set_collisionBehavior()

        if mode =='static':
            self.panda_moveit_wrap = PandaMoveGroupInterface()
            self.arm_group=self.panda_moveit_wrap._arm_group
            self.get_reference_pose()
        #publisher:
        # self.publisher_name = rospy.Publisher('topic_name',String)

        #subscriber:
        # self.subscriber_name = rospy.Subscriber('topic_name',String, self.subscriber_callback)
        
        # rate=rospy.Rate(800) 
        while not rospy.is_shutdown():
            self.control_loop(self.ref_position,self.ref_orientation)
            # break
            # rate.sleep()
    def set_collisionBehavior(self):
        self.collision=CollisionBehaviourInterface()
        torque_lower=[100.0]*7
        torque_upper=[100.0]*7
        force_lower=[100.0]*6
        force_upper=[100.0]*6
        self.collision.set_ft_contact_collision_behaviour(torque_lower,torque_upper,
                                                          force_lower,force_upper)

    def get_reference_pose(self,target_pose=None):
        #target_pose= geometry_msgs/Pose
        if self.mode=='static':
            self.go_static_test_pose()
            rospy.sleep(2.0)
            rospy.loginfo('---Moved to static test pose----')
            self.ref_position=se3_to_msg(self.limb.car_pose_trans_mat).position
            self.ref_orientation=se3_to_msg(self.limb.car_pose_trans_mat).orientation
            self.ref_velocities= np.array([0,0,0,0,0,0,0])
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
        #get coriolis
        coriolis=self.limb.get_coriolis()

        #get Jacobian
        J = self.limb.zero_jacobian()
        self.cur_poistion=se3_to_msg(self.limb.car_pose_trans_mat).position
        self.cur_orientation=se3_to_msg(self.limb.car_pose_trans_mat).orientation

        #compute deviation in position
        error=[]
        error.append(ref_position.x-self.cur_poistion.x)
        error.append(ref_position.y-self.cur_poistion.y)
        error.append(ref_position.z-self.cur_poistion.z)
        error.append(self.ref_orientation.x-self.cur_orientation.x)
        error.append(self.ref_orientation.y-self.cur_orientation.y)
        error.append(self.ref_orientation.z-self.cur_orientation.z)
        error=np.array(error).reshape(6,1)

        #compute deviation in velocity
        # #TODO: get reference velocity (for static = 0 )
        err_dot=[]
        #get joint velocities
        cur_dq=self.limb.joint_velocities()
        for name in self.joint_names:
            err_dot.append(cur_dq[name])
        err_dot=np.array(err_dot).reshape(7,1)
        d_error=np.dot(J,err_dot)

        #alternative: get_cartesian_velocity
        # cur_vel_trans=self.limb.endpoint_velocity()['linear']
        # cur_vel_rotat=self.limb.endpoint_velocity()['angular']
        # err_dot.append(cur_vel_trans[0]) #TODO: get reference velocity (for static = 0 )
        # err_dot.append(cur_vel_trans[1])
        # err_dot.append(cur_vel_trans[2])
        # err_dot.append(cur_vel_rotat[0])
        # err_dot.append(cur_vel_rotat[1])
        # err_dot.append(cur_vel_rotat[2])
        # d_error=np.array(err_dot).reshape(6,1)
        
        #TODO compute deviation in acceleration

        wrench=np.dot(self.P,error) - np.dot(self.D, d_error)
        tau_task=np.dot(J.T,wrench)
        #add coriolis
        tau_command=list(tau_task.reshape(7,1)+coriolis.reshape(7,1))

        # print(d_error)
        

       

        self.limb.set_joint_torques(dict(zip(self.joint_names,tau_command)))

    # def publish_function(self,arg):
        
    #     PLACEHOLDER
        
    #     self.publisher_name.publish(data)

    # def subscriber_callback(self,data):

    #     PLACEHOLDER

def main():
    rospy.init_node("robot_arm_controller")
    try:
        impedance_ctrl=Impedance_control()  #if program run from init, otherwise put starting function
        
    except rospy.ROSInterruptException:pass
        

    rospy.spin()
if __name__ == '__main__':
	main()
    