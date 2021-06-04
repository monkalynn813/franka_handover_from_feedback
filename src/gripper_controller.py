#!/usr/bin/env python
import rospy
from franka_interface import GripperInterface, ArmInterface
import numpy as np
import os , sys, select, tty, termios
import time
#import rosmsg needed:
from std_msgs.msg import String
from handover_controller.srv import enable_torque_controller
from handover_controller.srv import enable_gripper_controller
from create_webclient import read_to_file

read_to_file()
#load current policy sample:
sample_path ='/home/jingyan/Documents/handover_franka/handover_control_ws/src/handover_controller/sample/'
filename = 'sample.csv'
delimiter = ','
data=np.loadtxt(sample_path+filename,delimiter=delimiter)
dmax= data[0]
dmin = data[1]
lx = data[2]
ly = data[3]


class Gripper_controller():
    def __init__(self,width=0.054):
        self.joint_names=['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
        self.gripper=GripperInterface()
        self.limb=ArmInterface()
        self.width=width

        self.load_force_joint='panda_joint6'
        self.k=5.0
        self.limb.set_joint_position_speed(0.08)
        #publisher:
        # self.publisher_name = rospy.Publisher('topic_name',String)

        #subscriber:
        # self.subscriber_name = rospy.Subscriber('topic_name',String, self.subscriber_callback)
        self.controller_switch=False
        self.torque_controller_switch=rospy.ServiceProxy('torque_controller_switch',enable_torque_controller)
        rospy.Service('grippper_controller_switch',enable_gripper_controller,self.controller_status)
            
        self.experiment_loop()    
    def experiment_loop(self):
        self.initialization()
        read_to_file()
        #major experiment loop
        rate=rospy.Rate(50)
        epsilon=0.01

        self.pick_and_standby()
        try:
            impedance_status=self.torque_controller_switch(1)
            os.system('play -nq -t alsa synth {} sine {}'.format(0.07,1000))
        except Exception as e:
            rospy.logwarn(e)
        rospy.sleep(2.0)
        self.zero_load=self.read_baseline(2.0)
        rospy.sleep(2.0)

        while not rospy.is_shutdown():
            if self.controller_switch:
                self.command_position()  
            #detect if handover task has finished
            key=getKey()
            if key == '0':
                rospy.loginfo('------Handover Task Finished------')
                
                raw_input('Press Enter to start next round of the experiment')
                self.experiment_loop()
            
            rate.sleep()

    def initialization(self):
        self.gripper.stop_action()
        try:
            impedance_status=self.torque_controller_switch(0)
        except Exception as e:
            rospy.logwarn(e)
        
        self.controller_switch=False
    def read_baseline(self,sec):
        now = time.time()
        read =[]
        while time.time() <= now + sec:
            read.append(self.limb.joint_effort(self.load_force_joint))
        
        a = np.average(read)
        return a
    
    def controller_status(self,req):
        if req.gripper_controller_switch:
            self.controller_switch= True
            os.system('play -nq -t alsa synth {} sine {}'.format(0.09,500))
            rospy.loginfo('Enabling the gripper position controller')
        else:
            self.controller_switch= False
        return self.controller_switch

    def pick_and_standby(self):
        standby_positions=[np.pi/4, -np.pi/4, 0, -3*np.pi/4,0,np.pi,np.pi/4]
        # self.limb.move_to_joint_positions(dict(zip(self.joint_names,standby_positions)))
        # rospy.sleep(1)
        self.gripper.calibrate()
        
        
        self.limb.move_to_neutral()
        rospy.sleep(1.0)
        pick_positions=[-0.0342419010134475, 0.8901026895255373, 0.024786713402524006, -1.7151975027580983, 0.0024498268913563125, 2.4946672325929007, 0.7813774621688105]
        self.limb.move_to_joint_positions(dict(zip(self.joint_names,pick_positions)),use_moveit=False)

        epsilon=0.03
        success=self.gripper.move_joints(self.width-epsilon,wait_for_result=False)
        rospy.sleep(1.0)
        neutral=[-1.0165762574168886e-05, -0.7850608562837568, 0.000556157004417989, -2.356243464252405, 0.0007990042751352085, 1.572437513430913, 0.7852157028276059]
        self.limb.move_to_joint_positions(dict(zip(self.joint_names,neutral)),use_moveit=False)
        rospy.sleep(1.0)
        self.limb.move_to_joint_positions(dict(zip(self.joint_names,standby_positions)),use_moveit=False)
        rospy.sleep(1.0)
            
    def get_current_load(self):       
        cur_read=self.limb.joint_effort(self.load_force_joint)
        cur_load=cur_read-self.zero_load
        if abs(cur_load) < 1.5:
            cur_load = 0
        return cur_load
    def command_position(self):
        #Fl is positive when grasp something
        Fl=self.get_current_load()
        fminimum = self.width-0.01
        fmaximum = self.width+0.03
        fmin=self.width
        self.fwid=fmin-self.k*Fl
        if self.fwid < fminimum:
            self.fwid = fminimum
        if self.fwid > fmaximum:
            self.fwid = fmaximum
            self.gripper.stop_action()
            success=self.gripper.move_joints(self.fwid,wait_for_result=True)
        
        else: 
            success=self.gripper.move_joints(self.fwid,wait_for_result=False)      
        # print(self.fwid)

    def test_handover_position(self):
        joint_names=['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
        positions=[0, -np.pi/4, 0, -3*np.pi/4,0,np.pi,np.pi/4]
        self.limb.move_to_joint_positions(dict(zip(joint_names,positions)))
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
    rospy.init_node("gripper_controller")
    try:
        gripper_controller=Gripper_controller()  #if program run from init, otherwise put starting function
        
    except rospy.ROSInterruptException:pass
        

    rospy.spin()
if __name__ == '__main__':
	main()
  