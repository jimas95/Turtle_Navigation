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

        gms = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState)
        self.position = gms("turtlebot3_burger", "world").pose
        rospy.logerr(self.position)
        self.counter = 0
        # self.set_goal((0,0))
        self.loop()

    def loop(self):

        # self.move_base.cancel_goals_at_and_before_time()
        while not rospy.is_shutdown():
            rospy.logdebug("Loop")
            res = self.move_base.get_result()
            state = self.move_base.get_state()
            goal_state = self.move_base.get_goal_status_text()


            rospy.logdebug(state)
            rospy.logdebug(res)
            rospy.logdebug(goal_state)

            self.counter +=10
            if(self.counter>16 or state==3):
                rospy.logerr("Recalculate Frontriers ! ")

                self.counter = 0
                frontier_map = frontier(self.map,self.map_info,self.position)
                pos = frontier_map.frontier_world
                self.set_goal(pos)
                coord = self.from_coords_to_map()
                rospy.loginfo(coord)
                rospy.loginfo(self.position)
                temp = []
                for i in range(15):
                    makis = []
                    for j in range(15):
                        coords = (coord[0] + i-7,coord[1] + j-7)                        
                        makis.append(self.map[coords[0]][coords[1]])
                    temp.append(makis)
        
                for i in range(14,-1,-1):
                    rospy.loginfo(temp[i])
                rospy.logerr("MAKIS")

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




    def from_coords_to_map(self):
        if(self.map_info.resolution==0):
            rospy.logerr("ERROR map resolution is ZERO")
            return(0,0)
        return (int((self.position.position.y-self.map_info.origin.position.y)/self.map_info.resolution),
                int((self.position.position.x-self.map_info.origin.position.x)/self.map_info.resolution))


    def map_callback(self,msg):
        """
        map Topic Sercice 
        msg type OccupancyGrid
        update map * save map
        """

        # self.map =np.zeros([msg.info.height, msg.info.width])
        # temp = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        # for i in range(msg.info.height):
            # self.map[i] = temp[msg.info.height-i-1]
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

    def __init__(self,mapp,map_info,turtle_pos):
        
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
        for i in range(self.map_info.height):
            for j in range(self.map_info.width):
                frontier = (i,j)
                if(self.is_frontier(frontier)):
                    self.counter+= 1
                    self.find_nearest_frontier(frontier)
        
        #find real world coordinates for neareste frontier
        time = rospy.Time.now()
        self.frontier_world = self.map_to_world(self.nearest)

        rospy.logdebug("Frontiers Calculation DONE")
        rospy.logdebug("Frontiers Calculation time --> " + str(rospy.Time.now()-time))
        rospy.logdebug("Frontiers found --> " +str(self.counter))
        rospy.logdebug("Frontiers nearest coords--> " +str(self.nearest[0]) + "  " +str(self.nearest[1]) )
        rospy.logdebug("Robot       map   coords--> " +str(self.rob_pos[0]) + "  " +str(self.rob_pos[1]) )
        rospy.logdebug("Frontiers world   coords --> " +str(self.frontier_world))
        rospy.logdebug("Frontiers dist --> " +str(self.min_dist))

        temp = []
        for i in range(15):
            makis = []
            for j in range(15):
                coords = (self.rob_pos[0] + i-7,self.rob_pos[1] + j-7)
                # rospy.logdebug(coords)
                
                if(self.check_limits(coords)):
                    makis.append(self.map[coords[0]][coords[1]])
                    # rospy.logerr(self.map[coords[0]][coords[1]])
        # rospy.logerr(" ")
            temp.append(makis)
        
        for i in range(14,-1,-1):
            rospy.logerr(temp[i])

        # debug position 
        # from world --> map --> world1 --> map1 --> world2 --> map2
        world = turtle_pos
        map_pos = self.world_to_map(world)
        world1_pos = self.map_to_world(map_pos)
        world1_pos_pose = Pose(Point(world1_pos[0],world1_pos[1],0),Quaternion(0,0,0,1))

        map_pos1 = self.world_to_map(world1_pos_pose)
        world2_pos = self.map_to_world(map_pos1)
        world2_pos_pose = Pose(Point(world2_pos[0],world2_pos[1],0),Quaternion(0,0,0,1))
        map_pos2 = self.world_to_map(world2_pos_pose)

        rospy.logerr("world")
        rospy.logerr(world)
        rospy.logerr(map_pos)
        rospy.logerr(world1_pos)
        rospy.logerr(map_pos1)
        rospy.logerr(world2_pos)
        rospy.logerr(map_pos2)
        rospy.logerr("world")


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

        for i in range(3):
            for j in range(3):
                coords = (frontier[0] + i-1,frontier[1] + j-1)
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
        Input : pos --> Pose
        """
        pos_center_map_x = pos.position.y-self.map_info.origin.position.y
        pos_center_map_y = pos.position.x-self.map_info.origin.position.x
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