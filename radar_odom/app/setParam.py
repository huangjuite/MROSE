#!/usr/bin/env python3

import rospy
import numpy as np
import dynamic_reconfigure.client
from tf.transformations import euler_from_matrix

rospy.init_node('myconfig_py', anonymous=True)
client = dynamic_reconfigure.client.Client("rio_extrinsic")

sensor = rospy.get_param("/odom/radar")
T_r_i = rospy.get_param("/%s/T_r_i"%sensor)
T_r_i = np.array(T_r_i)
R_r_i = T_r_i[:3, :3]
euler = euler_from_matrix(R_r_i, 'sxyz')


params = { 'Rx' : euler[0], 'Ry' : euler[1], 'Rz' : euler[2] }
config = client.update_configuration(params)


