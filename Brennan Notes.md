# GVG and birds eye view notes
Brennan's record of changes and commands on the birds eye view and gvg projects.

---

## About

### Birds Eye View
This is a project that aims to allow a UGV and UAV to cooperate to co-localize and map environments. The goal is to let the UGV handle navigation, collision avoidance, and gmapping while the UAV handles the video feed for ORB-SLAM. The higher POV allows a better view of the environment.

### GVG
This is a project from a long time ago. The goal is to create a roadmap of a given area with meetpoints defining intersections and edges connecting with meetpoints. This is done real time by the robot driving along the center of the hallway and noticing when there are splits in the way it is traveling (The robot prefers to take road less travelled adn it makes all teh difference to it's efficiency).

---

## Log


### Before Jun 19 2017

* I did a bit to allow an easier interface to the program
	* Change user bearing selection to supply a list of bearings and allow the user to choose and index
		* this needs to mbe modified to allow a user to select any bearing, not just the unexplored ones
	* Modified a file to force the lidar to be available (modified from the original by shannon)
* launch simulator with `roslaunch gvg main_simulation.launch`
* launch physical robot with `roslaunch gvg main_turtlebot.launch`
* Figured out how to begin cooperative localization
	* `roslaunch afrl_driver turtlebot_20_hokuyo.launch`
		* Launches the turtlebot and lidar
	* `roslaunch bebop_tag_following bebop.launch`
		* Launches the UAV, joystick commands,usb camera, and tag identifier
	* `roslaunch bebop_tag_following controller.launch`
		* Launches tag follower
		* You may now take off and place the quad over the robot to where the tag is in view
	* `roslaunch bebop_tag_following orbslam.launch`
		* Launches Orbslam
	* `roslaunch cooperative_localization cl-complete.launch`
		* Launches gmapping and cooperative localization
	* `roslaunch gvg main_turtlebot.launch`
		* Launchest the main turtlebot with gvg following and mapping
	* WS1:
		* TERM1:
			* roslaunch afrl_driver turtlebot_20_hokuyo.launch & roslaunch bebop_tag_following bebop.launch
		* TERM2:
		 	* roslaunch bebop_tag_following controller.launch
		* TERM3:
			* roslaunch bebop_tag_following orbslam.launch
	* WS2:
		* TERM1:
			* roslaunch cooperative_localization cl-complete.launch
		* TERM2:
			* roslaunch gvg main_turtlebot.launch



### Mon Jun 19 2017

* Added `setLocal` and `setRemote` commands to allow a user to change whether the robot is being controlled disconnected or connected to teh internet. local is disconnected and remote is connected.
* Added filterOn parameter to the localizer to allow it to be easily disabled or enabled (default to disabled)
* found problem with lidar.
	* set ethernet to manual
	* ran `hokuyo_network_loader.sh` to fix the problem
	* added ip_address param to laser_node launch file with ip set to 192.168.1.20
* in the process of fixing the above, i broke the odometry for the simulation
* TODONE
	* fix odometry. I think it is the odometry in the GVGFollower.cpp thats messed up. It isnt reading the right topic.



### Tue Jun 20 2017

* Fixing odometry
	* odometry is not publishing to teh /indoor/gvg/odom_combined topic due to the filterOn variable
	* fixed implentation of filterOn as param, working in sim now
	* GVG graphic is also now launching correctly
* increased the closest distance threshold in gvg_maper.launch to 1.4 from .5 to eliminate fake meetpoints
* still an error due to lack of visibility that misses some meetpoints if they are out of the laser's view
	* This is an error in the sim and will become much worse with the reduced visilbity of the lidar on teh physcial robot
	* No solution that i see other than to stop when near a meetpoint and spin to find teh midpoint then move to teh meetpoint using dead reckoning, spin when there to see if actually a midpoint.
	* i believe that by lowering the threshold of being close enopugh tot he meetpoint i may be able to also fix it.
* Still a bug in the odometry in the hall. I am goint to try to diable the filter for physical robot and see where that gets me.



### Wed Jun 21 2017

* I narrowed the lidar range to more accurately simulate the robot => reducing to [-2,2] range
* I found teh source of teh vector out of bounds exceptiona dn i believe prevented it, after testing, it has not throw in a while

* Changing speed and angular speed
	* low and low turn
		* not maneuverable
	* low and normal turn
		* speed Pretty good
		* angle becomes off rather quickly
		* distance is good
		* finds all true meetpoints and disregards not real ones
* TODONE
	* Create a full map in the simulation then create full map with robot
	* ask about why doing this realtimne instead of mapping then gvging



### Thu Jun 22 2017

* I need to change how the borders are set to widen them and allow
	* to do this i changed teh MIN_RANGE_JUMP in LaserUtils.h .5->.7
	* i also increased epsilon .5->1
	* increased ROBOT_DIAMETER .5->.7
* Just as a note, none of the code is commented and it actually makes it really hard to read



### Wed Jun 28 2017

* Tested running sevearl times. Failed due to user input error. I need to be more careful about checking which node it thinks it at before confirming.



### Thu Jun 29 2017

* modified the Planner.ccp after i fount a `vector::_M_range_check` exception thrown due to teh planner not finding a route. I added a ROS_ERROR message in case it occurs again that breaks from the heuristic serach.
* Added map save using python pickle gvg_mapping/src/MapSaver.py
* TODONE: add map resume functionality by reading pickles and publishing to node and edge and probably more
* Make is a rosrun `rosrun gvg_mapper MapLoader.py `



### Fri Jun 30 2017

* Added a map loader that unpickles and calls the `gvg_mapper/LoadSavedMap` service
* Need to test both by running map saver, checking save file, then runnin map loader



### Mon Jul 3 2017

* I realize that i am incredibly dumb I found a preexisting map loader and saver that actually works in cpp
* map_handler loads and save maps
* I need to figure out how to edit saves in case i make a dumb mistake
* If i make a mistake i can just delete an edge or a node
* TODONE: Check on cpu usage across stage and gvg
* DEADLINE: July 10 have a full gvg map in simulation



### Thu Jul 6 2017

* Added a new vertex matching policy that only asks the user if it thinks it has found new vertex or if it is unsure of the vertex it is currently at
* Specified a `MAX_ERROR_AUTOMATCH` constant in GVGGraph.h that specifies the highest error allowed when matching
* Running what i hope is a full run of the simulator if I dont enter the wrong information anywhere
* I am running on matching policy 1 with user input bearing selector. I want to have full control over the robot for this simulation
* Currently at 1400s in and it has not faulted yet
* ERROR: The robot passed by the meetpoint and attempted to recover but failed.
* Attempting to recover the map by loading
* could not recover
* When approachign a meetpoint it should now go one-third speed
	* This makes higher exploration speeds safer and should detect more meetpoints when they are available
* Modified a do-while loop in `GVGFollower.cpp` to make it so it doesnt reach a good distance, sleep adn then be out of range.

* TODONE
	* Look for a weight on heuristic search for path finding
	* Look for normalization of distanc and uncertainty



### Fri Jul 7 2017

* Found `Astar_weight` in gvg_planner.launch
* Found `frontier_min_uncertainty_exploration` flag in gvg_planner.launch
* __Simulation code is working.__
	* All changes should be in the main_turtlebot.launch file from now on.



### Mon Jul 10 2017
* Fixed the hardware of the turtlebot in prep for getting gvg to work with it
* Placed standoffs under lidar to raise it above the camera allowing it to have full clear view of 270
* cleaned lidar to get rid of noise around the sides and ahead
* removed a tall layer from the turtle bot to make is half as tall
* Placed the wifi beacon on top so i would not lose connection
* Secured the camera
* connected the lidar to computer properly



### Tues Jul 11 2017

* Created seperate launch files for bringing up the bot and then starting gvg
* Found a bug in the Odometry (`/indoor/gvg/odom_combined`)
	* Covariance has incredibly large values that crash rviz and will probably lead to NaN in cpp
	* Tested a python script that removes them and rviz does not crash
	* Turns out the kobuki node publishes the outrageous values. Trying to find an online solution.
	* Maybe turning the filter on will fix the problem
	* Turning on the filter fixes twist but not pose



### Wed Jul 12 2017

* Found an issue for me. Rviz for kinetic **does not** support the laser viz messages used in laser utils



### Thu Jul 13 2017

* GVG was working mostly when testing, the issue is a bug with the lidar.
	* The scans from 600 to 700 have a lot of static that appears weirdly when walls obstacles are present. It shows that an object is around .6m which is wrong.
	* Averageing - Bad
		* Let L be the list of ranges from the scan
		* lower := sum(L[595:600])/len(L[595:600])
		* higher := sum(L[700:705])/len(L[700:705])
		* avg := (lower+higher)/2.0
		* for i in [600,700]
		*   L[i] <= avg
	* drawing a line from one side of the static to the other also didnt working
	* Maxpooling over the nearest 10 on each side when the range is < 1.5 works superbly
		* Not only does it smooth it, it doesnt affect things far away and beause it has a max range, it doesnt move things angularly much
		* Sticking with max pooling
	* the lidar fixer is under `laser_node` as `LaserCleaner.py`. it has been added to the launch file
* After Lunch i need to test GVG with the cleaner on
* TODONE: remap velocity inputs to /teleop_velocity_smoother/raw_cmd_vel to make it move more smoothly and not jerk around. This may help with odometry



### Mon Jul 17 2017

* Tested GVG code in teh hallway repeatedly
	* Faield each time
	* On inspection of rosbags, teh lidar had a very high large
	* Fixed the lag by removing the python files and adding the cleaning to the LaserUtils handleLaserScan method
* also need to make the max range of obstacles bigger, sometimes they arent found


### Mon Jul 24 2017

* Modified robot_node's relRotate method to use angular distance rather than time when doing turns. This shoudl fix startup moverment as well as meetpoint leaving behavior
* M<odified max pool to continue the pooling to the edges instead of going from 10 to len-10
* GVG code working physically, leaving so i dont break it.



### Tue Jul 25 2017

* Beginning work on the birds eye view stuff
	* Goals
		* Drone Dispatch
		* Drone Retrieval
	* Plan
		* Dispatch
			* Rotate in direction
			* Give linear velocity until out of camera view
			* drone navigates
		* retrieval
			* Using quad's camera, begin klooking for the turtlebot
			* Slowly rotate drone while panning camera up and down
			* When turtlebot registered, center horizontally in camera and try to keep centered vertically using pan
			* Visual servo to turtlebot
			* Stop when quadcopter in view
* Implemented Dispatch, needs testing
* Tested birds eye view, needs PID controller
* Added PID control needs adjusting
* yaw is good, z,y and x need increased Kd's
* logitech gamepad has low power, loses connection



### Mon Jul 31 2017

* Finished PId controllers last week and theorized the Retrieval and recovery processes
* Dispatch is implemented but i need to switch to using the PID controller for it
* Retrieval - long distance after dispatch
	* hover
	* Initialize tag recognition for quad camera
	* Slowly rotate looking for tag
	*	When tag in view
		* PID to keep centered
		* slowly approach
	* when tag lost
		* angular velocity = 0
		* forward velocity = low
	* when tag found by lower camera
		*	drop control
		* set odometry to match the lower bot and transform the map to match
* recovery - short distance after unexpected tag loss - implemented needs test
	* stop both bots
	*	find direction off bot that quad was last spotted at
	* do normal recovery assuming that direction with rotation transform
	* if does not recover after 5 seconds, call retrieval



### Tue Aug 1 2017

* cloned shannons old project and copied my stuff into it so i will be able to merge my changes into hers eventually
	* there is a brids-eye-view folder in the catkin_ws workspace that contains hwer code with my changes, all of which are in a git repo that allow changes to be shown adn it is merge ready whenever i want to do that
* reimplimented entire tag following system to allow smooth dispatch, retrieval, and long term loss recovery.
* TODO
	* test dispatch
	* test recovery
	* test retrieval
	* test long term recovery



### Wed Aug 2 2017

* Dispatch no longer works at all, no response to teh dispatch publish
* recovery leads to instability, change to stopping the quad adn making it go higher anmd turn the turtlebot



### Thu Aug 3 2017

* Fixed dispatch
* Fixed short term recovery
* Talked with alberto about the future of teh project, there are a couple of directions
	* Probabilistic approach to retreival and localization
		* Using a Particcle filter to figure out wher ethe UAV is relative to teh UGV
		* Keep the turtlebot exploring whil the UAV is exploring
		* Preserve communication if possible
	* Determine when to dispatch
		* When should the drone leave and when should it remain or return
	* probabilistic retrieval and localization is probably the best place to start so teh drone dispatch can be tested -IMO


### Mon Nov 6 2017

* remembered keeping a log is a thing to do, see slack for things realized before returning to the log
* rotors_simulator by ayushgard used for bebop simulation
	* contains bebop model
	* takes vx,vy,vz,vroll,vpitch,vyaw commands RollPitchYawrateThrust
	* bebop normally takes twist
	* created translator from twist to RollPitchYawrateThrust
* Working on adding turtlebot and getting cooperative localization workin in sim
* added a nice world to test in

### Thu Nov 16 2017
* Orbslam is having issues running

### Tue Nov 21 2017
* working on ORBSLAM again, seems like an issue with different versions of orbslam
* ORBSLAM works, needed to fully remove opencv-3.3.1
* gazebo is good, quad is running, orbslam is accurate
* Working on adding turtlebot according to the launch file from turtlebot_gazebo
* successfully added turtlebot, need to add camera to top and find how to control

### Wed Nov 22 2017
* turtlebot topics are broadcast but they dont actually control anything
* Camera on front works, will replace with lidar and add camera to top after i can get it moving.
* may be a diff between gazaebo 2.2 and 5

### Mon Nov 27 2017
* SDF error can be fixed by editing the xacro file "turtlebot_description/urdf/turtlebot_gazebo.urdf.xacro" and adding pointCloudCutoffMax after pointCloudCutoff
	* this doesnt fix the not being able to drive thing

### Sat Dec 2 2017
* http://answers.gazebosim.org/question/8371/kobukiturtlebot-doesnt-move-when-given-velocity-commands-in-gazebo-5/
	* This provides info on how to fix the issue
	* the kinematics of the turtle work when published to by the gz joint command method, but it it the translation to gazebo from ros that poses the issue.

### Mon Dec 4 2017
* `gz joint -m mobile_base -j wheel_left_joint --vel-t 1` sets the velocity of the wheel but i still cant find where the error in translation is
* TODO
	* Work through the process of translation and see where the error is

### Tue Dec 5 2017
* Message to alberto: Im stuck on the simulator again. publishing to the gazebo joints from command line works. the turtlebot manager node is launched. but publishing to the rostopic '/turtle/mobile_base/commands/velocity' or '/turtle/cmd_vel_mux/input/teleop' doesn't work. From the attached rqt_graph output, it is shown that the turtlebot's node manager does not connect to the gazebo node in ros
* Tested that velocity setting thing after setting all of the coefficients, it makes the turtlebot fly in a very Brownian way.
* The bebop connects directly to the gazebo node but the turtlebot does not. I think this is the issue
	* i will make a translator from turtlebot twist to joint commands using the force things because it seems a lot more stable than velocity
* I'll just take the time step and velocities and make the discreet steps.

### Fri Jan 12 2018
* Mono break is over
* Switching gears to work on the physical prototype to be able to write a paper possibly.
* implemented navigating to the object of interest
	* Here i will probably use an ar tag and say mission accomplished when it is within 1 meter
* began structuring navigating back to the turtle
	* Update waypoints after each map publishing
	* Update velocities after each orbslam update

## Tue Jan 23 2018
* Gears changed, now working on letting the quad rest on the jetyak and take off when it sees an object
* defined the controller header, pidcontroller
* need to find how to get the boat's GPS and what feature type to pursue
	* most likely starting with a n AR tag on a bridge or something
	* if i finish with enough time i could throw inception v3 on it and deploy to classify an object

## Wed Jan 24 2018
* Asked jason how to access teh boat's GPS signal
* Added to the headers for Controller and FeatureFinder
* Implemented BaseCamCallback and Controller constructor
* Wrote the CMakeList and package.xml

## Thur Jan 25
* Finished implementation for testing dispatch i think

## Fri Jan 26
* Unsure of whether to pursue the hex h20 so continuing with bebop
* Confirming will not do flight to be safe
* No control test went swimmingly
* Controller active flight test failed
	* the quad went in the wrong direction in the x,y direction on takeoff
	* angle of flight i wrong
## Sat Jan 27
* Dispatch works but needs more testing
* landing works but needs more testing and checking
* time to begin working of the GPS
*	created PID controllers for the landing seperate from teh travel PIDs
* set a landing threshold

## Mon Jan 29
* Dispatch works
* Landing works
* The upward facing camera needs aid in seeing the quad outside
	* leds on the corners of the platform
	* Larger tag
	* relaxed error contraints
* Must figure out what copter will be used soon

* Idea
	* for boat approach, have three tags at the top of the pole to navigate to the boat and avoid the possible
	* Have tags going down until the bottom cam find the quad
* TAG IDS FOR approach
	* quad ID and feature ID will be 0
	* There will be 6 tags to the top of teh possible
		* 1 at the bottom
		* 6 at the top
		* 7 will be next CCW from 6 and 8 will be next CCW from 7

## Thu Feb 1 2018
* Goal
	* Take off
		* Lift off
		* Visual tether to jetyak
		* Begin follow mode
	* Perform manuever
		* Circle the jetyak remaining visually tethered
	* Perform GPS based fly to other jetyak
	  * GPS navigate
		* Search for mast tags
	* Land
		* Use mast tags to get in the correct position
		* Point camera down and begin landing
		* At a certain height, drop and angle camera up
		* Have an AR tag in view to keep a visual tether while on the platform
* pkg
	* jetyak_uav_utils
		* controller
			* used to send commands to the quad
			* Implements a safety controller if mast becomes too close
			* Implements joystick override of other Nodes
			* listens to commands to switch modes
		* search_and_land
			* broadcasts which jetyak is in view
			* listens for which jetyak to search for
			* searches for mast ar tags
			* uses tags to line up for landing
			* uses downward facing camera to line up perfectly
			* sets down at certain height
		* take_off_and_follow
			* lifts off
				* give a z velocity
			* Uses downward facing camera to get to the height of the mast and face it
				* face camera down
				* Turn toward the mast
				* PID to the mast height
			* Uses forward camera to move behind
				* safely move around the mast using the tags
			* follow above the jetyak
				* keep tag f_x meters to the left, f_y meters ahead, and f_z meters above
		* cooperative_localization
			* Use shannons cl code
		* joy_control
			* Overrides other control
			* be able to
				* Land
				* give x,y,z,angular_z commands
