#!/usr/bin/env python
import rospy
from franka_interface import ArmInterface
from franka_moveit import PandaMoveGroupInterface 
from franka_tools import CollisionBehaviourInterface
from math import pi, sqrt
import numpy as np
from trajectory_generator import msg_to_se3, se3_to_msg
import time

#import rosmsg needed:
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from std_msgs.msg import String, Bool, Float64MultiArray
from handover_controller.srv import enable_torque_controller


class Impedance_control():
    def __init__(self,ptx=80,pty=80,ptz=100,pr=20,mode='static',tra_update_rate=1):
        """
        param:
        ptx,pty,ptz: translational stiffness at end-effector 
        pr: all rotatinal stiffness at end-effector
        tra_update_rate: the update rate of the receiving trajectory
        """
        #set stiffness:
        self.P = np.zeros([6,6])
        np.fill_diagonal(self.P,[ptx,pty,ptz,pr,pr,pr])
        #set damping: critical damp
        self.D= np.zeros([6,6])
        np.fill_diagonal(self.D,2*np.sqrt(np.diag(self.P)))

        self.mode=mode
        self.tra_update_rate=tra_update_rate

        self.joint_names=['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
        self.limb = ArmInterface()
        
        self.set_collisionBehavior()        

        if mode =='static':
            self.panda_moveit_wrap = PandaMoveGroupInterface()
            self.arm_group=self.panda_moveit_wrap._arm_group
            self.go_static_test_pose()
            self.get_reference_pose(mode)
            self.controller_switch=1

        #publisher:
        # self.publisher_name = rospy.Publisher('topic_name',String)

        #subscriber:
        if mode =='dynamic':
            #in case trajectory is not published yet
            # self.go_static_test_pose()            
            self.subscribe_flag=0
            self.controller_switch=0
            self.Trajectory_listener = rospy.Subscriber('robot_trajectory',Float64MultiArray, self.decompose_trajectory)
            #build a server to get indicator to start torque controller
            rospy.Service('torque_controller_switch',enable_torque_controller,self.controller_status)
        
        #initialize variables for acc computation
        self.previous_vel=[0.0]*7
        self.previous_timestamp=time.time()
        self.initial_acc_coeff=0

        self.enable_controller()

    def controller_status(self,indicator):
        if indicator==1:
            self.controller_switch = 1
        else:
            self.controller_switch = 0
        return self.controller_switch
            
    def enable_controller(self):
        #avoiding the loop starts at some random poses
        flag=True
        # start control loop 
        while not rospy.is_shutdown() and self.controller_switch:
            if flag:
                self.get_reference_pose(mode='static')
                rospy.loginfo('Enabling the impedance controller')
            if self.mode == 'dynamic':
                self.get_reference_pose(self.mode)
            self.control_loop(self.ref_pose,self.ref_vel,self.ref_acc)
            flag=False

    def set_collisionBehavior(self):
        self.collision=CollisionBehaviourInterface()
        torque_lower=[100.0]*7
        torque_upper=[100.0]*7
        force_lower=[100.0]*6
        force_upper=[100.0]*6
        self.collision.set_ft_contact_collision_behaviour(torque_lower,torque_upper,
                                                          force_lower,force_upper)
    def decompose_trajectory(self,traj):
        self.target_j_pos=traj.data
        self.subscribe_flag=1
                
        #compute time to move to next pose
        self.time = 1/self.tra_update_rate
        self.now=time.time()

    def get_reference_pose(self,mode):
        #target_pose= geometry_msgs/Pose
        if mode=='static':
            ref_position=se3_to_msg(self.limb.car_pose_trans_mat).position
            ref_orientation=se3_to_msg(self.limb.car_pose_trans_mat).orientation
            
            self.ref_pose=[ref_position,ref_orientation]
            self.ref_vel= np.array([0,0,0,0,0,0]).reshape(6,)
            self.ref_acc= np.array([0,0,0,0,0,0,0]).reshape(7,)
        
        if mode =='dynamic' and self.subscribe_flag==1:
            self.ref_pose=np.array(self.target_j_pos).reshape(7,)
            self.ref_vel= np.array([0,0,0,0,0,0,0]).reshape(7,)
            self.ref_acc= np.array([0,0,0,0,0,0,0]).reshape(7,)
            self.subscribe_flag=0

    def go_static_test_pose(self):
        rospy.loginfo("--moving to static test pose---")

        T_standby=np.array([[ 0,            0,          1,  4.29282581e-01],
                            [ -1/sqrt(2),  -1/sqrt(2),  0, 1.37536114e-04],
                            [ 1/sqrt(2),    -1/sqrt(2), 0, 6.84969607e-01],
                            [ 0,  0,  0, 1]])

        target_pose= se3_to_msg(T_standby)
        self.arm_group.set_pose_target(target_pose)
        self.arm_group.go(wait=True) 
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        rospy.sleep(2.0)
    
    def control_loop(self,ref_pose,ref_vel,ref_acc):
        #get coriolis
        coriolis=self.limb.get_coriolis()
        

        #get Jacobian
        J = self.limb.zero_jacobian()
        #get mass inertia matrix
        M = self.limb.joint_inertia_matrix()
        
        #in case get delta_joint_position; delta_joint_velocities; delta_joint_accelerations directly from the planner
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
                       
        #compute deviation in acceleration
        cur_timestamp=time.time()
        cur_joint_acc=(np.array(cur_joint_vel)-np.array(self.previous_vel))/(cur_timestamp-self.previous_timestamp)
        err_dotdot=np.array(ref_acc-cur_joint_acc).reshape(7,1)

        wrench=np.dot(self.P,error) + np.dot(self.D, err_dot) 
        tau_task=np.dot(J.T,wrench) + np.dot(M,err_dotdot)*self.initial_acc_coeff
            #add coriolis
        tau_command=list(tau_task.reshape(7,1)+coriolis.reshape(7,1))
        self.limb.set_joint_torques(dict(zip(self.joint_names,tau_command)))
        
        self.previous_vel=cur_joint_vel
        self.previous_timestamp=cur_timestamp
        self.initial_acc_coeff=0

def main():
    rospy.init_node("robot_arm_controller")
    try:
        impedance_ctrl=Impedance_control(mode='dynamic')  #if program run from init, otherwise put starting function

    except rospy.ROSInterruptException:pass
        

    rospy.spin()
if __name__ == '__main__':
	main()
    