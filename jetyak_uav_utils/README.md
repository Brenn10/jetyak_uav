# Jetyak UAV Utilities

***Aiming to provide an interface for the interaction of UAV and jetyaks***

## Nodes
* controller
	* used to send commands to the quad
	* Implements a safety controller if mast becomes too close - TODO
	* Implements joystick override of other Nodes
	* listens to commands to switch modes
* search_and_land
	* broadcasts which jetyak is in view
	* listens for which jetyak to search for
	* searches for mast ar tags
		* if mast detected, ensure it's pose matches the pose of the goal kayak
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

## External Libraries
* ar_track_alvar
	* find tags on boat
* robot_localization
	* Keep track of the pose of the quadcopter in relation to the kayak
	* integrate ar tag poses, imu, vo
	* Somehow gotta disable when dispatched and reset when landing
