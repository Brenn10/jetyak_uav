import rospy
from jetyak_uav_utils.msg import Waypoint, WaypointArray
from jetyak_uav_utils.srv import SetWaypoints,SetWaypointsResponse
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Joy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped
from math import atan2,cos,sin,pi,sqrt

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
def clip(val,low,high):
	return min(high,max(val,low))

class WaypointFollow():	
	def __init__(self,):
		self.deb_f = open("dump.cvs","w+")
		self.hover_alt=10
		self.flag = 0b00  # world frame and position commands
		self.wps = []
		self.yaw = 0
		self.lat = 0
		self.lon = 0
		self.height = 0
		self.max_speed=3
		self.in_waypoint = False
		self.time_entered_wp = 0

		rospy.init_node("waypoint_follower")
		self.cmd_pub = rospy.Publisher("/jetyak_uav_utils/extCommand", Joy, queue_size=1)
		self.yaw_sub = rospy.Subscriber("/dji_sdk/attitude",QuaternionStamped,self.att_callback,queue_size=1)
		self.gps_sub = rospy.Subscriber("/dji_sdk/gps_position", NavSatFix, self.gps_callback, queue_size=1)
		self.height_sub = rospy.Subscriber("/dji_sdk/height_above_takeoff", Float32, self.height_callback, queue_size=1)
		self.wps_service = rospy.Service("set_waypoints", SetWaypoints, self.wps_callback)

	def att_callback(self, msg):
		q = msg.quaternion
		orientation_list=[q.x,q.y,q.z,q.w]
		self.yaw = euler_from_quaternion(orientation_list)[-1]

	def gps_callback(self, msg):
		self.lat = msg.latitude
		self.lon = msg.longitude
		self.do_controls()

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

		if not self.in_waypoint:
			self.in_waypoint = self.check_if_in_wp()
			
		#If we are still in a waypoint
		if self.in_waypoint:
			#If this is our first time noticing we are in a waypoint
			if self.time_entered_wp == 0:
				self.time_entered_wp = rospy.Time.now().to_sec()
				self.deb_f.write("%f,%f\n"%(self.lon,self.lat))
				print("Entered waypoint")
			elif rospy.Time.now().to_sec() - self.time_entered_wp > self.wps[0].loiter_time:
				self.wps.pop(0)
				self.time_entered_wp = 0
				self.in_waypoint = False
				if(len(self.wps)==0):
					cmd = Joy()
					cmd.axes = [0,0, 0, 0, self.flag]
					#cmd.axes=[east,0,height_diff,0,self.flag]
					self.cmd_pub.publish(cmd)
					print("Mission Complete")
				else:
					print("Moving to next objective: %f left"%len(self.wps))
				return
		
		east = lat_lon_to_m(0, self.wps[0].lon, 0, self.lon)
		if(self.wps[0].lon<self.lon):
			east = -east
		north = lat_lon_to_m(self.wps[0].lat, 0, self.lat, 0)
		if(self.wps[0].lat<self.lat):
			north = -north
		#print("N: %1.5f, E: %1.5f"%(north,east))
		self.hover_alt = self.wps[0].alt

		height_diff=self.wps[0].alt-self.height

		mag= sqrt(east**2+north**2)
		if(mag>self.max_speed):
			east = (east/mag)*self.max_speed
			north= (north/mag)*self.max_speed

		cmd = Joy()
		cmd.axes = [east, north, height_diff, self.wps[0].heading, self.flag]
		#cmd.axes=[east,0,height_diff,0,self.flag]
		self.cmd_pub.publish(cmd)


	def check_if_in_wp(self,):
		dist = lat_lon_to_m(self.lat, self.lon, self.wps[0].lat, self.wps[0].lon)
		return pow(dist, 2) + pow(self.height - self.wps[0].alt, 2) < pow(self.wps[0].radius, 2)

wpfollow = WaypointFollow()
rospy.spin()
