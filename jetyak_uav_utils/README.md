# Jetyak UAV Utilities

***Aiming to provide an interface for the interaction of UAV and jetyaks***

## Prerequisites
Here listed a few dependencies that need to be downloaded from the corresponding
repository.

* DJI Onboard SDK
```
git clone https://github.com/dji-sdk/Onboard-SDK.git
cd Onboard-SDK
mkdir build
cd build
cmake ..
make
sudo make install
```

* DJI Onboard SDK ROS from the catkin_ws/src directory
```
git clone https://github.com/dji-sdk/Onboard-SDK-ROS
cd ..
catkin_make
```

## Nodes
* controller
	* used to send commands to the quad (y forward, x left, z-up)
	* Implements a safety controller if mast becomes too close
	* Implements joystick override of other Nodes
	* listens to commands to switch modes
	* TODO
		* Implement ar tag safety controller
* search
	* broadcasts which jetyak is in view
	* listens for which jetyak to search for
	* searches for mast ar tags
		* if mast detected, ensure it's pose matches the pose of the goal kayak
	* uses tags to line up for landing
	* TODO
		* Full implementation
* land
	* descends using ar tag bundle
	* TODO
		* Controller test
		Threshold test
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
	* TODO
		* full implementation

## External Libraries
* ar_track_alvar
	* find tags on boat
	* Uses tag bundle
			5
		 4 6
			3
			|
			1
			|
			0
	* tag 0 is master and reduces the effects of boat roll on measurements
* robot_localization
	* Keep track of the pose of the quadcopter in relation to the kayak
	* integrate the 6 ar tag poses
	* Somehow gotta disable when dispatched and restart when landing

## Needs
* Tags need to be mounted to the masts
* Strong router needed to connect the jetyaks GPS and UAV
	* test out configurations
* implementation needed for search and land Nodes
* Testing is needed for all current implementation
* for takeoff and follow, i am trying to figure out the transforms from ar_track_alvar to GROUND_ELU or BODY_FLU

## Joy reference
* LT or RT deadswitch
* B - Give Control
* A-land
* Y- Takeoff
* X - take control
* Right stick X - linear y
* Right stick Y - linear x
* left stick Y - linear z
* left stick X - Angular z
