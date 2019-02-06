import rospy as rp
import mavros_msgs.msg as mav_m


class Mavros_Bridge():
    def __init__(self,):
        self.mav_sub = rp.Subscriber(
            "/mavlink/from", mav_m.Mavlink, callback=self.mavlink_callback, queue_size=1000, tcp_nodelay=True)
        self.mav_pub = rp.Publisher(
            "/mavlink/to", mav_m.Mavlink, queue_size=10)
        rp.init_node("mavros_bridge")
