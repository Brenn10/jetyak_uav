# Place in the flight log directory of the robot


rm latest
ln -s /media/ubuntu/Manifold/FlightLog/`ls -rt /media/ubuntu/Manifold/FlightLog | tail -n1` latest
