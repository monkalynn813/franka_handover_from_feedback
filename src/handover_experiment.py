#!/usr/bin/env python
import rospy
#import controllers
from robot_arm_controller import Impedance_control, Handover_poses
from gripper_controller import Gripper_controller
from trajectory_generator import Trajectory_Generator

def handover_experiment(item_width):
    arm_joint_motion=Handover_poses()
    arm_torque_motion=Impedance_control(mode='dynamic')
    gripper_motion=Gripper_controller(item_width)
    reference_motion=Trajectory_Generator()

    #go handover standby pose and calibrate
    arm_joint_motion.go_standby_pose()
    gripper_motion.calibration()
    #go pickup item and standby
    arm_joint_motion.go_pickup_pose()
    gripper_motion.simple_grasp()
    arm_joint_motion.go_standby_pose()
    #enable impdance control loop
    arm_torque_motion.enable_controller()
    
    # gripper control loop, compute reference trajectory







def main():
    rospy.init_node("node_name")
    try:
        
        
    except rospy.ROSInterruptException:pass
        

    rospy.spin()
if __name__ == '__main__':
	main()
    