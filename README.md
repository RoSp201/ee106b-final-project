Final project starter notes
Robert Spark, Vidush Mukund, Gabriel Al-Harbi

starting up baxter
make a symbolic link in the root of your ros workspace:
	
	ln -s /scratch/shared/baxter_ws/baxter.sh ~/final_project/

log in to baxter shell in root of workspace. Make sure .bashrc is updated and source first if needed:

	./baxter.sh

should see yellow text with  http://robotbaxter.local:11311] in terminal prompt
NOTE: make sure baxter is turned on before logging into baxter shell in order to run commands!

enable baxter robot:

	rosrun baxter_tools enable_robot.py -e

untuck arms in to good starting configuration for motion planning:
	
	rosrun baxter_tools tuck_arms.py -u

if need to use cameras, make sure to open left_hand_camera and set resolution to max possible

	rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800

start joint trajectory action server in order to correctly publish joint velocity commands to baxter in new baxter shell:
	
	rosrun baxter_interface joint_trajectory_action_server.py

If using MoveIt to do motion planning, run the baxter moveit config launch script in a new shell:

	roslaunch baxter_moveit_config move_group.launch


Afterwards, run the following commands in this order while inside baxter:
	roslaunch freenect_launch freenect.launch
	roslaunch ar_track_alvar pr2_indiv.launch
	rosrun final_proj ar_track.py

The first command connects to the Kinect camera.
The second command connects AR tracking to the camera
The third command publishes AR tag positions and rotations based on custom translations.
