#!/usr/bin/env python3

import rospy
import numpy as np
import dynamic_reconfigure.client
from tf.transformations import euler_from_matrix

rospy.init_node('myconfig_py', anonymous=True)
client = dynamic_reconfigure.client.Client("rio_extrinsic")

sensor = rospy.get_param("/odom/radar")
params = {}

for i in range(len(sensor)):
    T_r_i = rospy.get_param("/%s/T_r_i" % sensor[i])
    T_r_i = np.array(T_r_i)
    R_r_i = T_r_i[:3, :3]
    euler = euler_from_matrix(R_r_i, 'sxyz')
    params.update({'R%dx' % i: euler[0], 'R%dy' %
                  i: euler[1], 'R%dz' % i: euler[2]})

config = client.update_configuration(params)
