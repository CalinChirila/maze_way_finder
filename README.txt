Project description:
This is a ROS project consisting of an obstacle avoiding robot (ROSbot).
There are 2 ways of operating the robot: manual or autonomous.
The project has been built in ROS Development Studio by theconstructsim.
The testing was made using the robot and the testing simulation provided by ROS Development Studio.

To build the project, from the catkin workspace run the command "catkin_make"
To launch the manual control mode, use the command "roslaunch maze_way_finder manual_control.launch"
To launch the autonomous mode, use the command "roslaunch maze_way_finder laser_scan.launch". When in the autonomous mode, the robot will move formward, when encountering an obstacle, it will slow down and turn in the direction in which the laser scan indicates more room. If the robot is very close to a wall, it will reverse.

To manually control the robot, use the wasd keys and press enter after each key in the same shell from which you launch the node. (The project accepts input from a user as part of the necessary operation of the program)

The "launch" folder in the project contains the launch files necessary for starting the nodes
The "src" folder contains 2 source files for the autonomous mode and manual control mode respectively

There is a Config.txt file in the package. This is used to configure different settings of the autonomous robot like angular speed, linear speed, reverse speed etc. (The project reads data from an external file or writes data to a file as part of the necessary operation of the program)
The project demonstrates an understanding of C++ functions and control structures.
At least two variables are defined as references, or two functions use pass-by-reference in the project code. (laser_scan.cpp lines 13 and 45)
The project uses multiple threads in the execution. (ROS Subscribers and publishers run on separate threads)