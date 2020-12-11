# homework04-jimas95

# Brief Overview 
The goal is to use the turtlebot to map an environment and then navigate within the map using the slam_toolbox.

# Package Architecture

## launch files

### start_slam
This launch file is executing SLAM algorithm in order to map the environment and then save it. In more detail we are using 
1. from pkg turtlebot3_gazebo the turtlebot3_house.launch environment.
2. from pkg turtlebot3_teleop the turtlebot3_teleop_key.launch in order to move the robot.
    Use wasdx for moving.
    1. w --> increase forward linear speed
    2. x --> decrease forward linear speed
    3. a --> increase forward angular speed
    4. d --> decrease forward angular speed
    5. s --> STOP
3. from pkg turtlebot3_slam turtlebot3_slam.launch for SLAM using gmapping
4. from pkg slam_toolbox online_async.launch for asynchronous SLAM
5. from pkg slam_toolbox online_sync.launch  for synchronous SLAM
6. from pkg turtlebot3_navigation turtlebot3_navigation.launch

#### Arguments
1. gazebo --> boolean, open gazebo and upload house environment, default FALSE
2. SLAM --> boolean, use synchronous/asynchronous SLAMING, default True
3. gmapping boolean, use gmapping for SLAM, default False

#### Execute
`$ export TURTLEBOT3_MODEL=${BURGER}`

`$ roslaunch turtle_slam start_slam.launch gazebo:=True`

If you want to save your map:
`$ rosrun map_server map_saver -f <map_name>`

rosrun map_server map_saver -f realHOUSE

### nav_stack
This launch file uses amcl to localize your robot and the ROS navigation stack to allow the robot to move using the map. You can move the robot by setting 2D navigation goals in Rviz.
1. from pkg turtlebot3_gazebo the turtlebot3_house.launch environment.
2. from pkg turtlebot3_navigation turtlebot3_navigation.launch and upload the map of the house(maps/map.yaml)

#### Arguments
1. gazebo --> boolean, open gazebo and upload house environment, default FALSE


#### Execute
$ export TURTLEBOT3_MODEL=${BURGER}

$ roslaunch turtle_slam nav_stack.launch gazebo:=True

![](https://github.com/ME495-EmbeddedSystems/homework04-jimas95/blob/main/gif/Navigation.gif)

### slam_stack
This launch file is used for slaming, the difference compared to start_slam is that now we use 2D navigation goals from Rviz to move the turtle robot.

1. from pkg turtlebot3_bringup the turtlebot3_remote.launch to upload the robot
2. from pkg turtlebot3_gazebo the turtlebot3_house.launch in order to upload the house scene
3. from pkg slam_toolbox the online_async.launch for SLAM 
4. from pkg turtlebot3_navigation the move_base.launch for moving the robot with 2D navigation goals 
5. from pkg rviz with configuration turtlebot3_navigation)/rviz/turtlebot3_navigation.rviz" for proper settings


#### Arguments
1. gazebo_world --> boolean, open gazebo and upload house environment, default FALSE
2. gazebo_house --> boolean, open gazebo and upload world environment, default FALSE


#### Execute
`$ export TURTLEBOT3_MODEL=${BURGER}`

`$  roslaunch turtle_slam slam_stack.launch gazebo:=True`


![](https://github.com/ME495-EmbeddedSystems/homework04-jimas95/blob/main/gif/SLAM.gif)




### explore
This launch file is able to autonoms navigate and explore at an unknown environment using SLAM and the frontier method. It utilizes the `slam_stack.launch` and has the same input arguments with it. The difference now is that we are using a node named `explore.pi` where we implement the frontier method. The robot might "stack" some times but using the 2D navigation goals we are able to easily help him get back on his corse. The node will end when all the environment has been explored
#### Arguments
1. gazebo_world --> boolean, open gazebo and upload house environment, default FALSE
2. gazebo_house --> boolean, open gazebo and upload world environment, default FALSE


#### Execute

## using gazebo ##
`roslaunch turtle_slam explore.launch gazebo_house:=True`


## real TurtleBot ##
1. `Open new Terminal`
2. `roscore`
3. `Open new Terminal`
4. `ssh ubuntu@turtlebot.local`
5. `cd ws/`
6. `source devel/setup.bash `
7. `roslaunch turtlebot3_bringup turtlebot3_robot.launch`
8. `Open new Terminal`
9. `ssh ubuntu@turtlebot.local sudo date -s @`(date -u +"%s" )` `
10. `roslaunch turtle_slam explore.launch `
