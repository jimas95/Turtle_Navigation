#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion,PoseStamped
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler

"""
This node will navigate at our enviroment and explore it while creating a map of it 
"""

class Explorer():

    def __init__(self):
        
        rospy.logdebug("Explorer initialize!")

        #create move_base action
        self.result = MoveBaseFeedback()
        self.move_base = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.logdebug("Wait for move_base server to go up")
        self.move_base.wait_for_server()
        rospy.logdebug("okei got that!")

        #subscibers 
        rospy.Subscriber('/move_base/feedback',MoveBaseAction,self.feedback)

        #test action
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        mygoal = Pose(Point(-0.5,-0.5,0),Quaternion(0,0,0,1))
        goal.target_pose.pose = mygoal
        self.move_base.send_goal(goal)

        self.loop()

    def loop(self):


        while not rospy.is_shutdown():
            rospy.logdebug("Explorer INIT")
            rate.sleep()


    def feedback(self,data):
        #print position 
        position = data.feedback.base_position.pose
        rospy.logdebug(position)
        euler_angles = self.get_rotation(position)
        self.print_euler_angles(euler_angles)

    def get_rotation(self,msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return (roll, pitch, yaw)

    def print_euler_angles(self,angles):
        rospy.logdebug("roll: " +str(angles[0]))
        rospy.logdebug("pitch: "+str(angles[1]))
        rospy.logdebug("yaw : "+str(angles[2]))
"""
Start everyting
"""
if __name__ == '__main__':
    try:
        rospy.init_node('Explorer', log_level=1)
        rate = rospy.Rate(1) # publish freacuancy (DO NOT Change)
        explore  = Explorer()
    except rospy.ROSInterruptException:
        pass