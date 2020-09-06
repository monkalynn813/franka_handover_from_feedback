#!/usr/bin/env python
import rospy
from franka_interface import GripperInterface, ArmInterface
import numpy as np
#import rosmsg needed:
from std_msgs.msg import String
from handover_controller.srv import enable_torque_controller
from handover_controller.srv import enable_gripper_controller



class Gripper_controller():
    def __init__(self,width=0.062):
        self.joint_names=['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
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
        self.controller_switch=False
        self.torque_controller_switch=rospy.ServiceProxy('torque_controller_switch',enable_torque_controller)
        rospy.Service('grippper_controller_switch',enable_gripper_controller,self.controller_status)
            
        self.experiment_loop()    
    def experiment_loop(self):
        #major experiment loop
        rate=rospy.Rate(100)
        epsilon=0.01

        self.pick_and_standby()
        impedance_status=self.torque_controller_switch(1)
        # self.test_handover_position()
        # self.calibration()
        # raw_input('---place bottle in the gripper position for test purpose----') 

        while not rospy.is_shutdown() and self.controller_switch:
            self.command_position()  
            #detect if handover task has finished
            if self.fwid > (self.width + epsilon):
                rospy.loginfo('------Handover Task Finished------')
                self.initialization()
                raw_input('Press Enter to start next round of the experiment')
                self.experiment_loop()
            
            rate.sleep()

    def initialization(self):
        self.gripper.stop_action()
        impedance_status=self.torque_controller_switch(0)
        self.controller_switch=False
    

    def controller_status(self,req):
        if req:
            self.controller_switch= True
            rospy.loginfo('Enabling the gripper position controller')
        else:
            self.controller_switch= False
        return self.controller_switch

    def pick_and_standby(self):
        standby_positions=[np.pi/2, -np.pi/4, 0, -3*np.pi/4,0,np.pi,np.pi/4]
        self.limb.move_to_joint_positions(dict(zip(self.joint_names,standby_positions)))
        rospy.sleep(1)
        self.calibration()
        
        pick_positions=[0, -np.pi/2, 0, -np.pi,0,np.pi,np.pi/4]
        self.limb.move_to_joint_positions(dict(zip(self.joint_names,pick_positions)))
        
        epsilon=0.001
        self.gripper.move_joints(self.width-epsilon,wait_for_result=True)
        rospy.sleep(0.5)

        self.limb.move_to_joint_positions(dict(zip(self.joint_names,standby_positions)))
        rospy.sleep(1.0)

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
        self.fwid=fmin-self.k*Fl
        self.gripper.move_joints(self.fwid,wait_for_result=False)       

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
    