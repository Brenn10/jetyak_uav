import rosbag
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
import sys
bag = rosbag.Bag(sys.argv[1])
x=[]
y=[]
z=[]
ts=[]
for topic, msg, t in bag.read_messages(topics=['/tag_pose']):
	x.append(msg.pose.position.x)
	y.append(msg.pose.position.y)
	z.append(msg.pose.position.z)
	ts.append(t.to_sec())
bag.close()

plt.plot(x,ts)
plt.plot(y,ts)
plt.plot(z,ts)
plt.show()
