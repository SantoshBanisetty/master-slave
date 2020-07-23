# master-slave

1. Create a package (catkin_create_pkg) named stage_ms and copy the files from this repository.
2. Open a new terminal and source the catkin_ws/devel/setup.bash file
```
source catkin_ws/devel/setup.bash
```
3. Execute the following command:
   ```
   roslaunch stage_ms OnYourMarkGetSetGo.launch
   ```
   You should see the sim.world file opened in the stage simulator and you will find two robots in it (Blue-Master and Green-Slave)
   You should see the blue robot wandering without hitting any obstacles and the green robot following the blue one and avoiding the obstracles along its path.
   
The link to the video demonstration is https://youtu.be/ono9Ie8n6ok

Happy Coding !! :-)

