import rospy
import numpy as np
import tf_conversions
import tf2_ros
from geometry_msgs.msg import TransformStamped

br = tf2_ros.TransformBroadcaster()
t = TransformStamped()

rospy.init_node('rviz_tf_rotate')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

rate = rospy.Rate(20.0)
i = 0
delta = 0.25
a = 0
r = 20
h = 5

while not rospy.is_shutdown():
    trans = TransformStamped()
    try:
        trans = tfBuffer.lookup_transform(
            "map", 'base', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue

    trans = [trans.transform.translation.x,
             trans.transform.translation.y,
             trans.transform.translation.z]
    a += delta

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "view"
    t.transform.translation.x = trans[0] + r * np.cos(np.deg2rad(a%360))
    t.transform.translation.y = trans[1] + r * np.sin(np.deg2rad(a%360))
    t.transform.translation.z = trans[2] + h

    
    q = tf_conversions.transformations.quaternion_from_euler(
        0, 0, np.deg2rad(a%360))
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    rate.sleep()
