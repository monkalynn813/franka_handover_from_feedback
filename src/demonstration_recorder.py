#!/usr/bin/env python
import rospy
from franka_interface import ArmInterface
from math import pi, sqrt
import numpy as np
import tf.transformations as tr
import tf_conversions.posemath as pm
import os , sys, select, tty, termios
from modern_robotics import IKinSpace
import time

#import rosmsg needed:
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from std_msgs.msg import Float64MultiArray
from handover_controller.srv import enable_gripper_controller
from handover_controller.srv import enable_torque_controller

#load current policy sample:
sample_path ='/home/jingyan/Documents/handover_franka/handover_control_ws/src/handover_controller/demonstration/'
filename = 'demo10.csv'
def writetofile(data,path):
	delim=','
	row=''
	for i in range(len(data)-1):
		row += str(data[i])
		row += delim
	row += str(data[-1])
	row += '\n'
	with open(path,'a') as f:
		f.write(row)
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
    def __init__(self,freq=10):
        """
        arguments:
            dmin: the distance indicates robot to move along with virtual human hand position
            dmax: the distance indicates robot to start move along with human hand position
            T_ch: the transformation matrix of human hand w.r.t. depth camera frame
        """
        self.joint_names=['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

        # self.dmax=dmax
        # self.dmin=dmin

        self.limb = ArmInterface()
        self.limb.set_joint_position_speed(0.08)
        
        self.T_sr= np.zeros([4,4])
        self.T_sm= np.zeros([4,4])
        self.T_r_r=np.array([[0,1,0,0], #actual robot frame wrt robot frame in optitrack sys
                             [0,0,1,0],
                             [1,0,0,0],
                             [0,0,0,1]])
        self.T_m_m=self.T_r_r
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

        self.freq=freq
        self.rate=rospy.Rate(freq)
        self.measure_timestp1=None
        self.gripper_flag=False
        self.robot_pose=self.hand_pose=self.man_hand=None
        self.damx_flag=False
        self.dmin_flag=False
        self.l_diag=False
        self.l_align=False

        #publisher:
        self.traj_publisher = rospy.Publisher('robot_trajectory',Float64MultiArray,queue_size=10)
        #subscriber:
        self.robot_pose=rospy.wait_for_message('vrpn_client_node/robot_pose/pose',PoseStamped).pose
        # self.robot_pose_sub=rospy.Subscriber('vrpn_client_node/robot_pose/pose',PoseStamped,self.robot_pose_callback)
        self.hand_pose_sub =rospy.Subscriber('vrpn_client_node/hand_pose/pose',PoseStamped,self.hand_pose_callback)
        

        self.man_hand_sub = rospy.Subscriber('vrpn_client_node/demon_hand/pose',PoseStamped,self.man_hand_callback)
        
        self.torque_controller_switch=rospy.ServiceProxy('torque_controller_switch',enable_torque_controller)

        flag=True
        self.old_target=None
        while not rospy.is_shutdown():

            if self.robot_pose!=None and self.hand_pose!=None and self.man_hand!=None:
                if flag:
                    self.get_robot_pose(self.robot_pose)
                    self.pick_and_standby()
                    try:
                        impedance_status=self.torque_controller_switch(1)
                        os.system('play -nq -t alsa synth {} sine {}'.format(0.07,1000))
                    except Exception as e:
                        rospy.logwarn(e)
                    rospy.sleep(2.0)
                    raw_input("Press 'Enter' to calibrate expert position")
                    self.calibratie_vitual_man_base()
                    flag=False
                self.compute_target_pose()
            self.rate.sleep()
            
    def robot_pose_callback(self,data):
        self.robot_pose=data.pose
    def hand_pose_callback(self,data):
        self.hand_pose=data.pose
    def man_hand_callback(self,data):
        self.man_hand=data.pose
    def get_robot_pose(self,robot_pose):        
        T_sr_=msg_to_se3(robot_pose)
        self.T_sr=np.dot(T_sr_,self.T_r_r)
    
    def pick_and_standby(self):
        standby_positions=[np.pi/4, -np.pi/4, 0, -3*np.pi/4,0,np.pi,np.pi/4]
        # self.limb.move_to_joint_positions(dict(zip(self.joint_names,standby_positions)))
        # rospy.sleep(1)
        # self.gripper.calibrate()
        
        
        # self.limb.move_to_neutral()
        # rospy.sleep(1.0)
        # pick_positions=[-0.0342419010134475, 0.8901026895255373, 0.024786713402524006, -1.7151975027580983, 0.0024498268913563125, 2.4946672325929007, 0.7813774621688105]
        # self.limb.move_to_joint_positions(dict(zip(self.joint_names,pick_positions)),use_moveit=False)

        # epsilon=0.03
        # success=self.gripper.move_joints(self.width-epsilon,wait_for_result=False)
        # rospy.sleep(1.0)
        # neutral=[-1.0165762574168886e-05, -0.7850608562837568, 0.000556157004417989, -2.356243464252405, 0.0007990042751352085, 1.572437513430913, 0.7852157028276059]
        # self.limb.move_to_joint_positions(dict(zip(self.joint_names,neutral)),use_moveit=False)
        # rospy.sleep(1.0)
        self.limb.move_to_joint_positions(dict(zip(self.joint_names,standby_positions)),use_moveit=False)
        rospy.sleep(1.0)

    def compute_target_pose(self):
        if self.T_sr.any() != np.zeros([4,4]).any() and self.T_sm.any() != np.zeros([4,4]).any(): 
            T_sh=msg_to_se3(self.hand_pose)
            T_rh=np.dot(np.linalg.inv(self.T_sr),T_sh)
            p_smh=np.array([self.man_hand.position.x,self.man_hand.position.y,self.man_hand.position.z,1]).reshape(4,1)
            p_mmh=np.dot(np.linalg.inv(self.T_sm),p_smh).reshape(4,)

            # dis=self.get_current_distance(T_rh)
            # # #get speed of human hand within certain distance measurement and assume it to be constant
            # speed=self.compute_hand_speed(dis)
            # if speed:
            #     rospy.loginfo("Hand speed is %0.3f m/s" %speed)

            #save ee and handposition data:
            p_re = self.limb.endpoint_pose()['position'].tolist()
            p_rh= T_rh[:3,-1].tolist()
            D=p_re+p_rh
            writetofile(D,sample_path+filename)

            #call IK solution to pub target joint positions

            if self.old_target is not None:
                noise = 0.01 #m
                deviation_sq=((self.old_target[0]-p_mmh[0])**2+(self.old_target[1]-p_mmh[1])**2+(self.old_target[2]-p_mmh[2])**2)
                if deviation_sq > noise**2:
                    self.traj_to_target_pose(p_mmh)
                    self.old_target=p_mmh
            else:
                self.traj_to_target_pose(p_mmh)
                self.old_target=p_mmh

              
    def calibratie_vitual_man_base(self):
        ################get SE3 of Tre############
        Tre_msg = Pose()

        px=self.limb.endpoint_pose()['position'][0]
        py=self.limb.endpoint_pose()['position'][1]
        pz=self.limb.endpoint_pose()['position'][2]
        ox=self.limb.endpoint_pose()['orientation'].x
        oy=self.limb.endpoint_pose()['orientation'].y
        oz=self.limb.endpoint_pose()['orientation'].z
        ow=self.limb.endpoint_pose()['orientation'].w
        Tre_msg.position.x=px
        Tre_msg.position.y=py
        Tre_msg.position.z=pz
        Tre_msg.orientation.x=ox
        Tre_msg.orientation.y=oy
        Tre_msg.orientation.z=oz
        Tre_msg.orientation.w=ow
        T_re = msg_to_se3(Tre_msg)
        T_se =np.dot(self.T_sr,T_re)
        T_se_msg = se3_to_msg(T_se)
        ##########################################
        #get position of man hand, set orientation same as actual ee
        T_smh_msg=self.man_hand
        T_smh_msg.orientation = T_se_msg.orientation
        T_smh = msg_to_se3(T_smh_msg)
   
        self.T_sm = np.dot(T_smh, np.linalg.inv(T_re))
    def compute_hand_speed(self,dis):
        epsilon=0.05
        measure_pt1=1.5 #m
        measure_pt2=0.8 #m
        speed=None
        
        if dis>= measure_pt1-epsilon and dis<=measure_pt1+epsilon:
            # print('reach the first measurement point')
            self.measure_timestp1=time.time()
        if dis>=measure_pt2-epsilon and dis<=measure_pt2+epsilon and self.measure_timestp1 is not None:
            # print('reach the second measurement point')
            measure_timestp2=time.time()
            speed=(measure_pt1-measure_pt2)/(measure_timestp2-self.measure_timestp1)
            self.measure_timestp1 = None
            measure_timestp2 = None
        return speed

        
    def traj_to_target_pose(self,hand_position): 
        # rospy.loginfo('following')  
        #compute distance:
        cur_joint_pos=[]
        #get current joint position
        cur_q=self.limb.joint_angles()
        for name in self.joint_names:
            cur_joint_pos.append(cur_q[name]) 
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
        l=0.0
        
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
    
    rospy.spin()
if __name__ == '__main__':
	main()
    