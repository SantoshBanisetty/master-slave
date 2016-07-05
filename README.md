# master-slave
assignment

1. Save the cave.png file in /opt/ros/indigo/share/stage_ros/world directory.
2. Create a package (catkin_create_pkg) named stage_first and copy the files from this repository.
3. run the following command:
   rosrun stage_ros stageros $(rospack find stage_ros)/world/sim.world
4. You should see the sim.world file opened in the stage simulator and you will find two robots in it (Blue-Master and Green-Slave)
5. Open a new terminal and source the catkin_ws/devel/setup.bash file
6. Execute the following command:
   roslaunch stage_first OnYourMarkGetSetGo.launch
   You should see the blue robot wandering without hitting any obstacles and the green robot following the blue one     and avoiding the obstracles along its path.
   
The link to the video demonstration is https://youtu.be/ono9Ie8n6ok

Happy Coding !! :-)

