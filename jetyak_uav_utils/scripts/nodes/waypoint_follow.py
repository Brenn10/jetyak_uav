import rospy
from jetyak_uav_utils.msg import Waypoint, WaypointArray
from jetyak_uav_utils.srv import SetWaypoints,SetWaypointsResponse
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Joy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped
from math import atan2,cos,sin,pi

def lat_lon_to_m(lat1,lon1,lat2,lon2):
	R = 6378137
	rLat1 = lat1 * pi / 180
	rLat2 = lat2 * pi / 180
	rLon1 = lon1 * pi / 180
	rLon2 = lon2 * pi / 180

	dLat = rLat2 - rLat1
	dLon = rLon2 - rLon1
	a = pow(sin(dLat / 2), 2) + cos(rLat1) * cos(rLat2) * pow(sin(dLon / 2), 2)
	return R * 2 * atan2(sqrt(a), sqrt(1 - a))

class WaypointFollow():	
	def __init__(self,):

		self.flag = 0b01  # world frame and position commands
		self.wps = []
		self.yaw = 0
		self.lat = 0
		self.lon = 0
		self.height = 0
		self.in_waypoint = False
		self.time_entered_wp = 0

		rospy.init_node("waypoint_follower")
		self.cmd_pub = rospy.Publisher("/jetyak_uav_utils/extCommand", Joy, queue_size=1)
		self.yaw_sub = rospy.Subscriber("/dji_sdk/attitude",QuaternionStamped,self.att_callback,queue_size=1)
		self.gps_sub = rospy.Subscriber("/dji_sdk/gps_position", NavSatFix, self.gps_callback, queue_size=1)
		self.height_sub = rospy.Subscriber("/dji_sdk/height_above_takeoff", Float32, self.height_callback, queue_size=1)
		self.wps_service = rospy.Service("set_waypoints", SetWaypoints, self.wps_callback)

	def att_callback(self, msg):
		orientation_q = msg.pose.pose.orientation
		orientation_list=[x for x in orientation_q]
		self.yaw = euler_from_quaternion(orientation_list)[-1]

	def gps_callback(self, msg):
		self.lat = msg.latitude
		self.lon = msg.longitude

	def height_callback(self, msg):
		self.height = msg.data

	def wps_callback(self, req):
		del self.wps[:]
		for i in req.waypoints.waypoints:
			print i
			self.wps.append(i)
		return SetWaypointsResponse(True)

	def do_controls(self,):
		if len(self.wps) == 0:
			return

		if not in_waypoint:
			in_waypoint = self.check_if_in_wp()
			
		#If we are still in a waypoint
		if in_waypoint:
			#If this is our first time noticing we are in a waypoint
			if self.time_entered_wp == 0:
				self.time_entered_wp = rospy.Time.now().to_sec()
			elif rospy.Time.now().to_sec() - self.time_entered_wp > self.wps[0].loiter_time:
				self.wps.pop()
				self.time_entered_wp = 0
				in_waypoint = False
		
		east = lat_lon_to_m(0, self.wps[0].lon, 0, self.lon)
		north = lat_lon_to_m(self.wps[0].lat, 0, self.lat, 0)
		height_diff = self.wps[0].height - self.height

		cmd = Joy()
		cmd.axes = [east, north, height_diff, self.wps[0].heading, self.flag]
		self.cmd_pub.publish(cmd)


	def check_if_in_wp(self,):
		dist = lat_lon_to_m(self.lat, self.lon, self.wps[0].lat, self.wps[0].lon)
		return pow(dist, 2) + pow(self.height - self.wps[0].alt, 2) < pow(self.wps[0].radius, 2)

wpfollow = WaypointFollow()
rospy.spin()
