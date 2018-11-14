# Place in the flight log directory of the robot


touch latest.ignore
ls -rt /media/ubuntu/Manifold/FlightLog | tail -n1 > latest.ignore
