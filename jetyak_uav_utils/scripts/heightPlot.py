import rosbag
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix,Imu
import sys

bag = rosbag.Bag(sys.argv[1])

lx=[]
ly=[]
lz=[]
ax=[]
ay=[]
az=[]
timu=[]

lat=[]
lon=[]
alt=[]
tgps=[]
'''
fimu = open("imu.csv","w+")
fgps = open("gps.csv","w+")
fimu.write("time,lx,ly,lz,ax,ay,az\n")
for topic, msg, t in bag.read_messages(topics=['/dji_sdk/imu']):
	lx.append(msg.linear_acceleration.x)
	ly.append(msg.linear_acceleration.y)
	lz.append(msg.linear_acceleration.z)
	ax.append(msg.angular_velocity.x)
	ay.append(msg.angular_velocity.y)
	az.append(msg.angular_velocity.z)
	timu.append(t.to_sec())
	fimu.write("%f,%f,%f,%f,%f,%f,%f\n"%(\
		t.to_sec(),msg.linear_acceleration.x,\
		msg.linear_acceleration.y,msg.linear_acceleration.z,\
		msg.angular_velocity.x,msg.angular_velocity.y,\
		msg.angular_velocity.z))

fgps.write("time,lat,lon,alt\n")
'''
for topic, msg, t in bag.read_messages(topics=['/dji_sdk/gps_position']):
	lat.append(msg.latitude)
	lon.append(msg.longitude)
	alt.append(msg.altitude)
	tgps.append(t.to_sec())
	'''
	fgps.write("%f,%f,%f,%f\n"%(\
		t.to_sec(),msg.latitude,msg.longitude,msg.altitude))
	'''

bag.close()

plt.plot(tgps,lat,'r')
plt.plot(tgps,lon,'g')
plt.plot(tgps,alt,'b')
plt.show()
