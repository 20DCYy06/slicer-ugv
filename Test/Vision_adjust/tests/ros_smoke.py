"""Run only in an isolated --network none container/private ROS master."""
from pathlib import Path
import sys
import time
import xmlrpc.client
import os
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vision_align import RAW_TOPIC, check_control_route, frame_is_fresh

rospy.init_node('vision_software_smoke', disable_signals=True)
bridge = CvBridge()
received = []

def receive(message):
    image = bridge.imgmsg_to_cv2(message, 'bgr8')
    received.append((image.shape, message.header.stamp.to_sec(), time.monotonic()))

sub = rospy.Subscriber('/vision_test/image', Image, receive, queue_size=1)
pub = rospy.Publisher('/vision_test/image', Image, queue_size=1)
raw = rospy.Publisher(RAW_TOPIC, Twist, queue_size=1)
end = time.monotonic() + 3
while not pub.get_num_connections() and time.monotonic() < end:
    time.sleep(.02)
assert pub.get_num_connections(), 'private Image subscriber did not connect'
message = bridge.cv2_to_imgmsg(np.zeros((120, 160, 3), dtype=np.uint8), 'bgr8')
message.header.stamp = rospy.Time.now()
pub.publish(message)
end = time.monotonic() + 2
while not received and time.monotonic() < end:
    time.sleep(.01)
assert received and received[-1][0] == (120, 160, 3), received
assert frame_is_fresh(received[-1][1], None, rospy.get_time(),
                      received[-1][2], time.monotonic(), .25)
master = rospy.get_master()
check_control_route(master.getSystemState(), rospy.get_name(), RAW_TOPIC)
foreign = '/vision_test_other_publisher'
foreign_uri = 'http://127.0.0.1:19999/'
raw_master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
reply = raw_master.registerPublisher(foreign, RAW_TOPIC, 'geometry_msgs/Twist', foreign_uri)
assert reply[0] == 1, reply
try:
    try:
        check_control_route(master.getSystemState(), rospy.get_name(), RAW_TOPIC)
    except ValueError as exc:
        assert foreign in str(exc), str(exc)
    else:
        raise AssertionError('foreign raw publisher accepted')
finally:
    raw_master.unregisterPublisher(foreign, RAW_TOPIC, foreign_uri)
    raw.unregister()
    sub.unregister()
    pub.unregister()
    rospy.signal_shutdown('offline checks complete; no actuator node')
print('PASS: private ROS Image/cv_bridge/freshness and real MasterProxy route checks; no actuator node')
