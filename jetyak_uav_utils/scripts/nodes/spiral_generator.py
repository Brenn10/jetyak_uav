import rospy
from jetyak_uav_utils.msg import Waypoint, WaypointArray
from jetyak_uav_utils.srv import SetWaypoints, SetWaypointsResponse
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Joy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped
from math import atan2, cos, sin, pi, sqrt

def frange(start, stop, parts):
	L=[]
	step = (stop - start) / parts
	for i in range(parts):
		L.append(start + step * i)
	return L
		
class SpiralGenerator():
	def __init__(self, wp_radius, endradius, turns, points,altitude):
		self.wp_radius = wp_radius
		self.endradius = endradius
		self.turns = turns
		self.points = points
		self.altitude = altitude
		self.wp_radius = wp_radius
		
		
		
		rospy.init_node("spiral_generator")

		self.center_service = rospy.Subscriber("create_spiral", NavSatFix, self.spiral_callback)
		self.set_wps_service = rospy.ServiceProxy("set_waypoints", SetWaypoints)
		
	def spiral_callback(self, msg):
		r = frange(0,self.endradius,self.points)
		t = frange(0, 2 * pi * self.turns, self.points)
		x = msg.latitude
		y = msg.longitude
		waypoints=[]
		for i in range(self.points):
			wp = Waypoint()
			wp.alt = self.altitude
			wp.lat = y + r[i] * sin(t[i])
			wp.lon = x + r[i] * cos(t[i])
			wp.radius = 5
			wp.loiter_time = 0
			waypoints.append(wp)
		waypoints_srv = SetWaypoints()
		waypoints_srv.waypoints=waypoints
		self.set_wps_service(waypoints_srv)
		print("Spiral Pattern Sent")
		
		

wp_radius = 1
endradius = .0005
turns = 5
points = 1000
altitude = 10

spiraler = SpiralGenerator(wp_radius, endradius, turns, points, altitude)
rospy.spin()
