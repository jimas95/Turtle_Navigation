#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion,PoseStamped
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
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


        self.map = np.array([])
        self.map_info = OccupancyGrid().info


        #subscibers 
        rospy.Subscriber('/move_base/feedback',MoveBaseAction,self.feedback_callback)
        rospy.Subscriber('/map',OccupancyGrid,self.map_callback)

        #test action
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        mygoal = Pose(Point(-0.5,-0.5,0),Quaternion(0,0,0,1))
        goal.target_pose.pose = mygoal
        self.move_base.send_goal(goal)

        self.loop()

    def loop(self):

        # self.move_base.cancel_goals_at_and_before_time()
        while not rospy.is_shutdown():
            rospy.logdebug("Loop")
            res = self.move_base.get_result()
            state = self.move_base.get_state()
            goal_state = self.move_base.get_goal_status_text()

            if(state!=3):
                rospy.logerr("WOW ! got something weird on state")
                rospy.logerr(state)
            rospy.logdebug(state)
            rospy.logdebug(res)
            rospy.logdebug(goal_state)



            rospy.loginfo("map data")
            
            rospy.loginfo(self.map_info.resolution)
            rospy.loginfo(self.map_info.width)
            rospy.loginfo(self.map_info.height)
            rospy.loginfo(self.map_info.origin)
            rospy.loginfo(self.map_info.resolution*self.map_info.width)
            rospy.loginfo(self.map_info.resolution*self.map_info.height)
            rospy.loginfo(self.from_coords_to_map())
            rospy.loginfo((self.position.position.x,self.position.position.y))
            rospy.logerr(self.map.shape)
            # for row in self.map:
            #     rospy.loginfo(row)

            rate.sleep()


    def create_frontier(self):
        pass

    def from_coords_to_map(self):
        return (int((self.position.position.x-self.map_info.origin.position.x)/self.map_info.resolution),
                int((self.position.position.y-self.map_info.origin.position.y)/self.map_info.resolution))


    def map_callback(self,msg):
        """
        map Topic Sercice 
        msg type OccupancyGrid
        update map * save map
        """

        self.map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
        # rospy.loginfo(data.info)
        # rospy.loginfo(data.data)

    def feedback_callback(self,data):
        self.position = data.feedback.base_position.pose
        # euler_angles = self.get_rotation(self.position)
        # rospy.logdebug(position)
        # self.print_euler_angles(euler_angles)

    def get_rotation(self,msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return (roll, pitch, yaw)

    def print_euler_angles(self,angles):
        rospy.logdebug("roll: " +str(angles[0]))
        rospy.logdebug("pitch: "+str(angles[1]))
        rospy.logdebug("yaw : "+str(angles[2]))



class frontier:

    def __init__(self,map,map_info):
        pass

    def is_frontier(self):
        pass



    def find_nearest_frontier(self):
        pass

    
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