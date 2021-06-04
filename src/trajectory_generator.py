#!/usr/bin/env python
import rospy
from franka_interface import ArmInterface
from math import pi, sqrt
import numpy as np
import tf.transformations as tr
import tf_conversions.posemath as pm
from modern_robotics import IKinSpace
import time
import os , sys, select, tty, termios
#import rosmsg needed:

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from std_msgs.msg import Float64MultiArray
from handover_controller.srv import enable_gripper_controller
from create_webclient import read_to_file


def get_new_parameters():
    read_to_file()
    sample_path ='/home/jingyan/Documents/handover_franka/handover_control_ws/src/handover_controller/sample/'
    filename = 'sample.csv'
    delimiter = ','
    data=np.loadtxt(sample_path+filename,delimiter=delimiter)
    dmax= data[0]
    dmin = data[1]
    lx=data[2]
    ly=data[3]
    print('\n load new parameters:',dmax, dmin, lx, ly)
    return dmax, dmin, lx, ly

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
        
        self.dmax,self.dmin,self.lx,self.ly=get_new_parameters()

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

        self.freq=freq
        self.rate=rospy.Rate(freq)
        self.measure_timestp1=None
        self.gripper_flag=False
        self.robot_pose=self.hand_pose=None

        #publisher:
        self.traj_publisher = rospy.Publisher('robot_trajectory',Float64MultiArray,queue_size=10)
        self.gripper_controller_switch=rospy.ServiceProxy('grippper_controller_switch',enable_gripper_controller)
        #subscriber:
        rospy.sleep(2.0)
        
        self.robot_pose=rospy.wait_for_message('vrpn_client_node/robot_pose/pose',PoseStamped).pose
        self.hand_pose_sub =rospy.Subscriber('vrpn_client_node/hand_pose/pose',PoseStamped,self.hand_pose_callback)

        flag=True
        self.old_target=None
        self.constant_moving_counter=0
        while not rospy.is_shutdown():

            if self.robot_pose!=None and self.hand_pose!=None:

                if flag:
                    self.get_robot_pose(self.robot_pose)
                    flag=False
                self.compute_target_pose(self.hand_pose)    
                self.check_completion()        
            else:
                rospy.loginfo('no optitrack info')
            
            self.rate.sleep()
            
    # def robot_pose_callback(self,data):
    #     self.robot_pose=data.pose
    def hand_pose_callback(self,data):
        self.hand_pose=data.pose
    def get_robot_pose(self,robot_pose):        
        T_sr_=msg_to_se3(robot_pose)
        self.T_sr=np.dot(T_sr_,self.T_r_r)
        
    def compute_target_pose(self,hand_pose):
        if self.T_sr.any() != np.zeros([4,4]).any():    
            T_sh=msg_to_se3(hand_pose)
            T_rh=np.dot(np.linalg.inv(self.T_sr),T_sh)
            p_rh=T_rh[:3,-1]
            dis=self.get_current_distance(p_rh)
            
            #get speed of human hand within certain distance measurement and assume it to be constant
            # speed=self.compute_hand_speed(dis)
            # if speed:
            #     rospy.loginfo("Hand speed is %0.3f m/s" %speed)
                #TODO send speed to policy

            #call IK solution to pub target joint positions
            if dis<=self.dmax:               
                if dis>self.dmin:
                    if self.old_target is not None:
                        noise = 0.01 #m
                        deviation_sq=((self.old_target[0]-p_rh[0])**2+(self.old_target[1]-p_rh[1])**2+(self.old_target[2]-p_rh[2])**2)
                        if deviation_sq > noise**2:
                            self.traj_to_target_pose(p_rh)
                            self.old_target=p_rh
                    else:
                        self.traj_to_target_pose(p_rh)
                        self.old_target=p_rh                    

                elif dis<=self.dmin:                    
                    if not self.gripper_flag: 
                        try:
                            gripper_status=self.gripper_controller_switch(True)
                            self.gripper_flag=True
                        except Exception as e:
                            rospy.logwarn(e)
                    #move constantly at 5cm /s
                    if self.constant_moving_counter <4: #for safety purpose        
                        target_pos=self.old_target
                        target_pos[0] -= 0.08
                        self.traj_to_target_pose(target_pos)
                        self.constant_moving_counter+=1
                        self.old_target=target_pos
            else:
                self.gripper_flag=False       

              
        
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

    def check_completion(self):
        key=getKey()
        if key == '0':
            rospy.loginfo('------Handover Task Finished------')
            
            raw_input('Press Enter to start next round of the experiment')
            self.gripper_flag=False
            self.dmax,self.dmin,self.lx,self.ly=get_new_parameters()

    def traj_to_target_pose(self,hand_position): 
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
        R=np.array([[ xscrew[0],    yscrew[0],  zscrew_x],
                    [ xscrew[1],    yscrew[1],  zscrew_y],
                    [ xscrew[2],    yscrew[2],  0,      ]])

        theta=pi/4
        Rrot=np.array([[np.cos(theta), -np.sin(theta), 0],
                       [np.sin(theta), np.cos(theta),  0],
                       [0,              0,             1]])
        #matain certain distance between hand and gripper

        #rotate along z axis of the gripper frame by 45 degree
        target_rotM=np.dot(R,Rrot)

        target_pose=np.append(target_rotM,[[0,0,0]],axis=0)
        target_pose=np.append(target_pose,np.array([[x+self.lx,y+self.ly,z,1]]).reshape(4,1),axis=1)

        joint_target,success=IKinSpace(self.Slist,self.M,target_pose,np.array(cur_joint_pos),0.01,0.005)
        if success:
            message=Float64MultiArray()
            message.data=joint_target
            self.traj_publisher.publish(message)
            # print('following')
        else:
            rospy.logwarn('Cannot solve IK frm current position')


    def get_current_distance(self,hand_pose):
        #compute current distance between human hand and end-effector
        current_T_be_position = self.limb.endpoint_pose()['position']
        d_square=0
        for i in range(2):
            d_square+=(current_T_be_position[i]-hand_pose[i])**2
        return sqrt(d_square)
def getKey():

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
    return key   
def main():
    rospy.init_node("trajectory_generator")
    try:
        trajectory_generator=Trajectory_Generator()  #if program run from init, otherwise put starting function
    
    except rospy.ROSInterruptException:pass
    
    rospy.spin()
if __name__ == '__main__':
	main()
    