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
import time

#import rosmsg needed:
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, PoseArray
from  moveit_msgs.msg import RobotTrajectory
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import String, Bool



class Impedance_control():
    def __init__(self,ptx=150,pty=150,ptz=150,pr=10,mode='static',tra_update_rate=1):
        """
        param:
        ptx,pty,ptz: translational stiffness at end-effector 
        pr: all rotatinal stiffness at end-effector
        tra_update_rate: the update rate of the receiving trajectory (consistent with hand pose update rate)
        """
        #set stiffness:
        self.P = np.zeros([6,6])
        np.fill_diagonal(self.P,[ptx,pty,ptz,pr,pr,pr])
        #set damping: critical damp
        self.D= np.zeros([6,6])
        np.fill_diagonal(self.D,2*np.sqrt(np.diag(self.P)))
        #TODO: set inertia
        

        self.mode=mode
        self.tra_update_rate=tra_update_rate

        self.joint_names=['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
        self.limb = ArmInterface()
        
        self.set_collisionBehavior()

        
        self.panda_moveit_wrap = PandaMoveGroupInterface()
        self.arm_group=self.panda_moveit_wrap._arm_group

        if mode =='static':
            self.go_static_test_pose()
            self.get_reference_pose(mode)


        #publisher:
        # self.publisher_name = rospy.Publisher('topic_name',String)

        #subscriber:
        if mode =='dynamic':
            #in case trajectory is not published yet
            self.go_static_test_pose()
            self.get_reference_pose(mode='static')
            self.subscribe_flag=0
            self.Trajectory_listener = rospy.Subscriber('robot_trajectory',PoseArray, self.decompose_trajectory)
        
        # rate=rospy.Rate(800) 
            
        while not rospy.is_shutdown():
            if mode == 'dynamic':
                self.get_reference_pose(mode)
            self.control_loop(self.ref_pose,self.ref_vel)
        #     # break
            # rate.sleep()
    def set_collisionBehavior(self):
        self.collision=CollisionBehaviourInterface()
        torque_lower=[100.0]*7
        torque_upper=[100.0]*7
        force_lower=[100.0]*6
        force_upper=[100.0]*6
        self.collision.set_ft_contact_collision_behaviour(torque_lower,torque_upper,
                                                          force_lower,force_upper)
    def decompose_trajectory(self,traj):
        self.subscribe_flag=1
        self.traj_poses=traj.poses
        
        #compute time to move to next pose
        self.period = 1./(self.tra_update_rate*len(self.traj_poses))
        self.pose_index=0
        self.now=time.time()

    def get_reference_pose(self,mode):
        #target_pose= geometry_msgs/Pose
        if mode=='static':
            ref_position=se3_to_msg(self.limb.car_pose_trans_mat).position
            ref_orientation=se3_to_msg(self.limb.car_pose_trans_mat).orientation
            
            self.ref_pose=[ref_position,ref_orientation]
            self.ref_vel= np.array([0,0,0,0,0,0]).reshape(6,)
        
        if mode =='dynamic' and self.subscribe_flag ==1:
            if time.time()-self.now >= self.period and self.pose_index<len(self.traj_poses)-1:
                    self.pose_index+=1
                    self.now=time.time()

            target_pose=self.traj_poses[self.pose_index]
            current_pose=se3_to_msg(self.limb.car_pose_trans_mat)
            self.ref_pose=[target_pose.position,target_pose.orientation]
           
            x_vel=(target_pose.position.x-current_pose.position.x)/self.period
            y_vel=(target_pose.position.y-current_pose.position.y)/self.period
            z_vel=(target_pose.position.z-current_pose.position.z)/self.period
            qua_x_vel=(target_pose.orientation.x-current_pose.orientation.x)/self.period
            qua_y_vel=(target_pose.orientation.y-current_pose.orientation.y)/self.period
            qua_z_vel=(target_pose.orientation.z-current_pose.orientation.z)/self.period

            self.ref_vel=np.array([x_vel,y_vel,z_vel,qua_x_vel,qua_y_vel,qua_z_vel]).reshape(6,)
             # self.ref_vel= np.array([0,0,0,0,0,0]).reshape(6,)
    def go_static_test_pose(self):
        rospy.loginfo("--moving to static test pose---")

        # T_standby=np.array([[-0.1815108 , -0.98090692,  0.06982443, -0.15],
        #                     [ 0.00259123,  0.07052656,  0.99750654,  0.5],
        #                     [-0.98338554,  0.18123914, -0.01025958,  0.5],
        #                     [ 0.        ,  0.        ,  0.        ,  1. ]])

        T_standby=np.array([[1/sqrt(2) , -1/sqrt(2),  0, 0.31],
                        [ -1/sqrt(2), -1/sqrt(2), 0,   0],
                        [0,  0, -1,  0.52],
                        [ 0, 0, 0,  1 ]])
        target_pose= se3_to_msg(T_standby)
        self.arm_group.set_pose_target(target_pose)
        self.arm_group.go(wait=True) 
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        rospy.sleep(2.0)
    
    def control_loop(self,ref_pose,ref_vel):
        #get coriolis
        coriolis=self.limb.get_coriolis()

        #get Jacobian
        J = self.limb.zero_jacobian()
        
        #in case get delta_joint_position; delta_joint_velocities directly from the planner
        if type(ref_pose) != list:
            #compute deviation in joint positions
            cur_joint_pos=[]
            #get current joint position
            cur_q=self.limb.joint_angles()
            for name in self.joint_names:
                cur_joint_pos.append(cur_q[name]) 
            
            delta_q=(ref_pose-np.array(cur_joint_pos)).reshape(7,1)
            error=np.dot(J,delta_q).reshape(6,1)

            #compute deviation in joint velocities      
            cur_joint_vel=[]
            #get joint velocities
            cur_dq=self.limb.joint_velocities()
            for name in self.joint_names:
                cur_joint_vel.append(cur_dq[name]) 

            delta_q_dot=(ref_vel-np.array(cur_joint_vel)).reshape(7,1)
            err_dot=np.dot(J,delta_q_dot).reshape(6,1)

            #TODO: add internia term

            
        else:
            ref_position=ref_pose[0]
            ref_orientation=ref_pose[1]
            ref_ee_vel=ref_vel

            #get current ee cartesian pose
            cur_poistion=se3_to_msg(self.limb.car_pose_trans_mat).position
            cur_orientation=se3_to_msg(self.limb.car_pose_trans_mat).orientation
            
            #compute deviation in position
            error=[]
            error.append(ref_position.x-cur_poistion.x)
            error.append(ref_position.y-cur_poistion.y)
            error.append(ref_position.z-cur_poistion.z)
            error.append(ref_orientation.x-cur_orientation.x)
            error.append(ref_orientation.y-cur_orientation.y)
            error.append(ref_orientation.z-cur_orientation.z)
            error=np.array(error).reshape(6,1)

            #compute deviation in velocity         
            cur_joint_vel=[]
            #get joint velocities
            cur_dq=self.limb.joint_velocities()
            for name in self.joint_names:
                cur_joint_vel.append(cur_dq[name])      
            cur_ee_vel=np.dot(J,np.array(cur_joint_vel).reshape(7,1))
            err_dot=(ref_ee_vel-np.array(cur_ee_vel).reshape(6,)).reshape(6,1)
                       
            #TODO compute deviation in acceleration

        wrench=np.dot(self.P,error) + np.dot(self.D, err_dot)
        tau_task=np.dot(J.T,wrench)
            #add coriolis
        tau_command=list(tau_task.reshape(7,1)+coriolis.reshape(7,1))
        self.limb.set_joint_torques(dict(zip(self.joint_names,tau_command)))


def main():
    rospy.init_node("robot_arm_controller")
    try:
        impedance_ctrl=Impedance_control(mode='dynamic')  #if program run from init, otherwise put starting function
        
    except rospy.ROSInterruptException:pass
        

    rospy.spin()
if __name__ == '__main__':
	main()
    