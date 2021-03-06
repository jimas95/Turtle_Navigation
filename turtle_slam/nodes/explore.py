#!/usr/bin/env python
import rospy
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseGoal, MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion,PoseStamped
import actionlib
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import time
from gazebo_msgs.srv import GetModelState
from nav_msgs.srv import GetMap
import tf2_ros

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


        # services
        self.get_map = rospy.ServiceProxy("/slam_toolbox/dynamic_map",GetMap)
        
        #init turtle position
        self.position = (0,0)

        # init map
        self.map = np.array([])
        self.map_info = OccupancyGrid().info

        #create buffer for tf transforms
        self.buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.buffer)

        #first update
        self.update()

        self.counter = 0
        self.loop()
        

    def loop(self):
        """
        main loop iterates every 1 sec 
        every 6 secs or if movement has been completed we recalculate new frontiers and set new goal
        """
        while not rospy.is_shutdown():

            rospy.logdebug("Loop")
            state = self.move_base.get_state()

            self.counter +=1
            if(self.counter>6 or state==3):
                rospy.logdebug("-------------------------")
                rospy.logdebug("Recalculate Frontriers ! ")
                rospy.logdebug("-------------------------")

                self.counter = 0
                frontiers_num = self.update()

                #break condition
                if frontiers_num==0 :
                    rospy.logdebug("---------------------------------------")
                    rospy.logdebug("---------------------------------------")
                    rospy.logdebug("NO FRONTIERS FOUND EXPLORATION COMPLETE")
                    rospy.logdebug("---------------------------------------")
                    rospy.logdebug("---------------------------------------")
                    break



            rate.sleep()


    def update(self):
        """
        main update of loop
        """
        #update position
        trans = self.buffer.lookup_transform("map", "base_footprint", rospy.Time(),rospy.Duration(1))
        self.position = (trans.transform.translation.x,trans.transform.translation.y)
        #update map
        
        #update map
        self.map_callback(self.get_map().map)

        #update forntiers
        frontier_map = frontier(self.map,self.map_info,self.position)
        pos = frontier_map.frontier_world
        #set goal
        self.set_goal(pos)

        #check if there are any frontiers left
        return frontier_map.counter

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



    def map_callback(self,msg):
        """
        Updates our map
        msg type OccupancyGrid
        update map * save map
        """
        self.map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info






class frontier:

    def __init__(self,mapp,map_info,turtle_pos):
        
        #keep track of time execution
        time = rospy.get_time()
        #save map info
        self.map_info = map_info
        self.map = mapp
        #transform robot coord to map coordinates
        self.rob_pos = self.world_to_map(turtle_pos)

        #init nearest as the farthest point
        self.nearest = (10000,10000)
        self.min_dist = abs(self.rob_pos[0]-self.nearest[0]) + abs(self.rob_pos[1]-self.nearest[1])

        #calculate all frontiers and find nearest
        self.counter = 0 
        #find all possible frontiers                  
        for i in range(self.map_info.height):
            for j in range(self.map_info.width):
                frontier = (i,j)
                if(self.is_frontier(frontier)):
                    self.counter+= 1
                    self.find_nearest_frontier(frontier)


        #find real world coordinates for neareste frontier
        self.frontier_world = self.map_to_world(self.nearest)
        rospy.logdebug("Frontiers Calculation DONE")
        rospy.loginfo("Frontiers Calculation time --> " + str(rospy.get_time()-time))
        rospy.logdebug("Frontiers found --> " +str(self.counter))
        rospy.logdebug("Robot       map   coords--> " +str(self.rob_pos[0]) + "  " +str(self.rob_pos[1]) )
        rospy.logdebug("Frontiers   goal  coords--> " +str(self.nearest[0]) + "  " +str(self.nearest[1]) )
        rospy.logdebug("Frontiers   goal  coords --> " +str(self.frontier_world))
        rospy.logdebug("Frontiers dist --> " +str(self.min_dist))
        rospy.logdebug("Print my local map")
        self.print_local_map()



    def print_local_map(self):
        """
        print a small part of map with the robot in center of it
        """
        size = 15
        size_half = int(size/2)
        temp_map = []
        for i in range(size):
            map_row = []
            for j in range(size):
                coords = (self.rob_pos[0] + i-size_half,
                          self.rob_pos[1] + j-size_half)      

                if(self.check_limits(coords)):
                    if self.rob_pos[0]==coords[0] and self.rob_pos[1]==coords[1]:
                        map_row.append("R")
                    else:
                        map_row.append(self.map[coords[0]][coords[1]])
            temp_map.append(map_row)
        
        #print map upside down cause thats how its saved....
        for i in range(14,-1,-1):
            rospy.logdebug(temp_map[i])

    def is_frontier(self,frontier):
        """
        Check if map(frontier) is a frontier
        Input: map --> np.array(height,width)
               frontier --> tuple(x,y)
        Return False if it is not Frontier
               True  if it is     Frontier
        """

        wall = False
        unexplored = False
        available = False
        size = 7
        size_half = int(size/2)
        if self.map[frontier[0]][frontier[1]]!=-1: return False
        for i in range(size):
            for j in range(size):
                coords = (frontier[0] + i-size_half,frontier[1] + j-size_half)
                if(self.check_limits(coords)):
                    if(self.map[coords[0]][coords[1]]==100): wall = True
                    if(self.map[coords[0]][coords[1]]==-1): unexplored = True
                    if(self.map[coords[0]][coords[1]]==0): available = True

        if wall : return False
        if not unexplored : return False
        if not available : return False
        return True

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
        pos_x = map_pos[1]*self.map_info.resolution+self.map_info.origin.position.x
        pos_y = map_pos[0]*self.map_info.resolution+self.map_info.origin.position.y
        return (pos_x,pos_y)

    def world_to_map(self,pos):
        """
        given pos position calculate the position at map coordinates
        Input : pos --> tuple(x,y)
        """
        pos_center_map_x = pos[1]-self.map_info.origin.position.y
        pos_center_map_y = pos[0]-self.map_info.origin.position.x
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
        if dist < self.min_dist and 15 < self.min_dist:
            self.min_dist = dist
            self.nearest = frontier
    
"""
Start everyting
"""
if __name__ == '__main__':
    try:
        rospy.init_node('Explorer', log_level=1)
        # rospy.init_node('Explorer')
        rate = rospy.Rate(1) # publish freacuancy (DO NOT Change)
        explore  = Explorer()
    except rospy.ROSInterruptException:
        pass