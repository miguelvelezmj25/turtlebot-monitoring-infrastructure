# Instruction graph action server

Based on the work by Andrew Bensen at Carnegie Mellon University.
Additions by Anurag Kanungo, Bradley Schmerl and Ekansh Gupta.

## Setup

- Clone the project using:
```
git clone https://github.com/ekanshgupta90/ig_action_server.git
```

- Move the files to your catkin workspace src directory.
```
cp -r ./* <catkin_ws>/src/
```

by default
```
cp -r ./* ~/catkin_ws/src/
```

- Make the source file (you need to be in catkin workspace directory):
```
cd ..

catkin_make
```

- Add the build files to environment:
```
source devel/setup.bash
```

- Run rospack to add the packages to roscd:
```
rospack depends1 ig_action_server

rospack depends1 ig_action_client
```

## Running

- Run ROS Gazebo
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```

- Run AMCL
```
roslaunch turtlebot_gazebo amcl_demo.launch
```

- Run Rviz
```
roslaunch turtlebot_rviz_launchers view_navigation.launch
```

- Run IG Server
```
roscd ig_action_server/src
python ig_server.py
```

- Run IG Client
```
roscd ig_action_client/src
python ig_client.py instructions/new.ig
```

