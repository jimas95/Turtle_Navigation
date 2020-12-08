#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion,PoseStamped
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import time
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
            fro = frontier(self.map,self.map_info,self.position)

            # for row in self.map:
            #     rospy.loginfo(row)

            rate.sleep()



    def set_goal(self,pos):
        """
        set goal position for turtle, 
        Input pos --> tuple(x,y)
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        mygoal = Pose(Point(pos[0],pos[1],0),Quaternion(0,0,0,1))
        goal.target_pose.pose = mygoal
        self.move_base.send_goal(goal)


    def create_frontier(self):
        rospy.logdebug("Recalculating Frontiers....")
        
        pass

    def from_coords_to_map(self):
        if(self.map_info.resolution==0):
            rospy.logerr("ERROR map resolution is ZERO")
            return(0,0)
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

    def __init__(self,map,map_info,turtle_pos):
        
        #save map info
        self.map_info = map_info
        self.map = map
        #transform robot coord to map coordinates
        self.rob_pos = self.world_to_map(turtle_pos)

        #init nearest as the farthest point
        self.nearest = (map_info.height,map_info.width)
        self.min_dist = abs(self.rob_pos[0]-self.nearest[0]) + abs(self.rob_pos[1]-self.nearest[1])

        #calculate all frontiers and find nearest
        self.counter = 0 
        for i in range(self.map_info.height):
            for j in range(self.map_info.width):
                frontier = (i,j)
                if(self.is_frontier(frontier)):
                    self.counter+= 1
                    self.find_nearest_frontier(frontier)
        
        #find real world coordinates for neareste frontier
        self.frontier_world = self.map_to_world(self.nearest)

        rospy.logdebug("Frontiers Calculation DONE")
        rospy.logdebug("Frontiers found --> " +str(self.counter))
        rospy.logdebug("Frontiers nearest coords--> " +str(self.nearest[0]) + "  " +str(self.nearest[1]) )
        rospy.logdebug("Frontiers woord   coords --> " +str(self.frontier_world))
        rospy.logdebug("Frontiers dist --> " +str(self.min_dist))


    def is_frontier(self,frontier):
        """
        Check if map(frontier) is a frontier
        Input: map --> np.array(height,width)
               frontier --> tuple(x,y)
        Return False if it is not Frontier
               True  if it is     Frontier
        """
        if self.map[frontier[0]][frontier[1]] != -1 : return False
        temp = (frontier[0]+1,frontier[1])
        if self.check_limits(temp) and self.map[temp[0]][temp[1]]==0: return True
        temp = (frontier[0]+1,frontier[1]+1)
        if self.check_limits(temp) and self.map[temp[0]][temp[1]]==0: return True
        temp = (frontier[0]+1,frontier[1]-1)
        if self.check_limits(temp) and self.map[temp[0]][temp[1]]==0: return True
        temp = (frontier[0]-1,frontier[1])
        if self.check_limits(temp) and self.map[temp[0]][temp[1]]==0: return True
        temp = (frontier[0]-1,frontier[1]+1)
        if self.check_limits(temp) and self.map[temp[0]][temp[1]]==0: return True
        temp = (frontier[0]-1,frontier[1]-1)
        if self.check_limits(temp) and self.map[temp[0]][temp[1]]==0: return True
        temp = (frontier[0],frontier[1]+1)
        if self.check_limits(temp) and self.map[temp[0]][temp[1]]==0: return True
        temp = (frontier[0],frontier[1]-1)
        if self.check_limits(temp) and self.map[temp[0]][temp[1]]==0: return True
        return False


    def check_limits(self,frontier):
        """
        check if frontier exeeds map limits
        Input frontier --> tuple(x,y)
        Output : False for exeeding limits
                 True its okei
        """

        if(frontier[0]+1>self.map_info.height or frontier[0]<0): return False
        if(frontier[1]+1>self.map_info.width or frontier[1]<0): return False
        return True

    def map_to_world(self,map_pos):
        """
        Transform map position to real world coordinates
        Input map_pos --> tuple(x,y)
        Return tuple(x,y)
        """
        pos_x = map_pos[0]*self.map_info.resolution+self.map_info.origin.position.x
        pos_y = map_pos[1]*self.map_info.resolution+self.map_info.origin.position.y
        return (pos_x,pos_y)

    def world_to_map(self,pos):
        """
        given pos position calculate the position at map coordinates
        Input : pos --> Pose
        """
        pos_center_map_x = pos.position.x-self.map_info.origin.position.x
        pos_center_map_y = pos.position.y-self.map_info.origin.position.y
        pos_center_map_x = pos_center_map_x/self.map_info.resolution
        pos_center_map_y = pos_center_map_y/self.map_info.resolution
        pos_center_map_x = int(pos_center_map_x )
        pos_center_map_y = int(pos_center_map_y)
        return (pos_center_map_x,pos_center_map_y)

    def find_nearest_frontier(self,frontier):
        """
        Calculate distance of given frontier
        if less the keep that as the nearest
        Input frontier --> tuple(x,y)
        """
        #calculate manhatan distance 
        dist = abs(self.rob_pos[0]-frontier[0]) + abs(self.rob_pos[1]-frontier[1])
        if dist < self.min_dist:
            self.min_dist = dist
            self.nearest = frontier
    
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