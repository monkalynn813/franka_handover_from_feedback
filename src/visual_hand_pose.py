#!/usr/bin/env python
import rospy
import tf.transformations as tr
import tf_conversions.posemath as pm
import numpy as np
from math import sqrt
#import rosmsg needed:
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped

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

class hand_pose():
    def __init__(self):
        
        self.radius= 0.15
        self.circle_center=[0,0.65] #(x,z)
        self.acc=3
        self.x=self.circle_center[0]-self.radius
        self.z= sqrt(self.radius**2-(round(self.x-self.circle_center[0],self.acc))**2)+self.circle_center[1]
        self.switch=1

        self.T_bc= np.array([[1,0,0,1],
                            [0,1,0,0.5],
                            [0,0,1,0],
                            [0,0,0,1]])

        #publisher:
        self.publisher_name = rospy.Publisher('visual_hand_pose',Pose,queue_size=20)

        #subscriber:
        # self.subscriber_name = rospy.Subscriber('topic_name',String, self.subscriber_callback)

        rate=rospy.Rate(30)
        rospy.loginfo("-------press Enter to start generate visual human hand pose-----")
        raw_input()
        while not rospy.is_shutdown():
            self.publish_function() 

            rate.sleep()
    def publish_function(self):
        if round(self.x,self.acc) == round(self.circle_center[0]+self.radius,self.acc):
            self.switch=-1
        if round(self.x,self.acc)==round(self.circle_center[0]-self.radius,self.acc):
            self.switch=1
        self.x+=self.switch*3e-3
        self.z= self.switch*sqrt(self.radius**2-(round(self.x-self.circle_center[0],self.acc))**2)+self.circle_center[1]
        
        T_bh=np.array([[-0.1815108 , -0.98090692,  0.06982443, self.x],
                        [ 0.00259123,  0.07052656,  0.99750654,  0.5],
                        [-0.98338554,  0.18123914, -0.01025958,  self.z],
                        [ 0.        ,  0.        ,  0.        ,  1. ]])
        T_ch=np.dot(np.linalg.inv(self.T_bc),T_bh)
        self.publisher_name.publish(se3_to_msg(T_ch))

    # def subscriber_callback(self,data):

    #     data+=1

def main():
    rospy.init_node("node_name")
    try:
        visual_hand_pose=hand_pose()  #if program run from init, otherwise put starting function
        
    except rospy.ROSInterruptException:pass
        

    rospy.spin()
if __name__ == '__main__':
	main()
    