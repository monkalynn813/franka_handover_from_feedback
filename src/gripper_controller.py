#!/usr/bin/env python
import rospy
from franka_interface import GripperInterface, ArmInterface
import numpy as np
#import rosmsg needed:
from std_msgs.msg import String



class Gripper_controller():
    def __init__(self,width=0.062):
        self.gripper=GripperInterface()
        self.limb=ArmInterface()
        self.width=width

        self.load_force_joint='panda_joint6'
        self.k=0.013
        self.F_ovl=30


        #publisher:
        # self.publisher_name = rospy.Publisher('topic_name',String)

        #subscriber:
        # self.subscriber_name = rospy.Subscriber('topic_name',String, self.subscriber_callback)
        
        rate=rospy.Rate(100)

        self.test_handover_position()
        self.calibration()

        raw_input('---place bottle in the gripper position for test purpose----') 

        while not rospy.is_shutdown():
            self.command_position()
    
            rate.sleep()
     


    def set_slope_parameter(self,slope):
        self.k=slope
    
    def calibration(self):
        self.gripper.calibrate()
        self.zero_load=self.limb.joint_effort(self.load_force_joint)


    def get_current_load(self):       
        cur_read=self.limb.joint_effort(self.load_force_joint)
        cur_load=cur_read-self.zero_load
        return cur_load
    def command_force(self):
        #Fl is positive when grasp something
        Fl=self.get_current_load()
        Fg=self.k*Fl+self.F_ovl
        self.gripper.grasp(self.width,Fg,epsilon_inner=0.05,epsilon_outer=0.05,wait_for_result=False)
    def command_position(self):
        #Fl is positive when grasp something
        Fl=self.get_current_load()
        fmin=self.width
        fwid=fmin-self.k*Fl
        self.gripper.move_joints(fwid,wait_for_result=False)
    def simple_grasp(self):
        epsilon=0.001
        self.gripper.move_joints(self.width-epsilon,wait_for_result=True)

    def test_handover_position(self):
        joint_names=['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
        positions=[0, -np.pi/4, 0, -3*np.pi/4,0,np.pi,np.pi/4]
        self.limb.move_to_joint_positions(dict(zip(joint_names,positions)))
    
def main():
    rospy.init_node("gripper_controller")
    try:
        gripper_controller=Gripper_controller()  #if program run from init, otherwise put starting function
        
    except rospy.ROSInterruptException:pass
        

    rospy.spin()
if __name__ == '__main__':
	main()
    