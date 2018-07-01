# How to use

## Using Move Base
1. Run ```catkin_make``` in the workspace folder
2. Run ```roslaunch aitech_task gazebo.launch``` in first terminal.
3. Run ```roslaunch aitech_task move_base.launch``` in second terminal.
4. Run ```roslaunch aitech_task task.launch``` in third terminal.

This is a standard ROS catkin workspace. Node Laser2PC processes incoming Laser scan data and converts it to point cloud data. Using this point cloud data the coordinates of the legs of the table are identified in the Odom frame. And finally, the pose of the center of the table is determined which is published as a visualization message Marker. The other node SendGoal receives this pose of the center of the table and sends it as a goal to move_base action server. The navigation stack is setup that takes care of the goal point.

The result of the change of pose of the table is reflected with a marker in Rviz can be seen
[here](https://youtu.be/toN-7hoduYQ).\
Here is the result of the above package [Table Docking Demo](https://youtu.be/8NRbqzYfsAo).

## Using simple P controller
1. Run ```catkin_make``` in the workspace folder
2. Run ```roslaunch aitech_task gazebo.launch``` in first terminal.
3. Run ```roslaunch aitech_task rviz_navigation.launch``` in second terminal.
4. Run ```roslaunch aitech_task docking.launch``` in third terminal.

The Omnibot first tries to set its position and orientation in front of table and once its done it try to get inside the table.

Here is the result of [Table Docking with P controller](https://youtu.be/kduseSWBnKM).
