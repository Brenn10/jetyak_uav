#!python

import rospy as rp
from sensor_msgs.msg import Joy
from jetyak_uav_utils.msg import SetString


class RC_Interpreter():

	def __init__(self, minD, maxD, parts):

		self.modeTable = ["land", "takeoff", "leave",
				  "land", "follow", "return",
				  "hover", "follow", "hover"]

		# parameter stuff
		self.minD = minD
		self.maxD = maxD
		self.range = maxD-minD
		self.parts = parts

		# ros stuff
		rp.init_node("rc_interpreter")
		self.joySub = rp.Subscriber("/mavros/rc/in", joyCallback, queue_size=1)
		rp.wait_for_service("/jetyak_uav_utils/setMode"
		self.modeCient=rp.ServiceProxy("/jetyak_uav_utils/setMode", SetString)
		rp.spin()

	def joyCallback(self, msg):
		part=int(round((float(msg.axes[8]-self.minD)/self.range)*parts))
		srv=SetString
		srv.data=self.modeTable[part]
		print(self.modeClient(srv))

if __name__ == "__main__":
	rc=RC_Interpreter(900, 2000, 9)
