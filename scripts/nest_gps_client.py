#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
import gps		# the gpsd interface module
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix
from nest.srv import NestGPSMessage, NestGPSMessageRequest

if __name__ == '__main__':
	rospy.init_node('gps_client')
	rospy.wait_for_service('/nest1_gps')
	nest_gps_service_client = rospy.ServiceProxy('/nest1_gps',NestGPSMessage)
	nest_gps_service_object = NestGPSMessageRequest()

	result = nest_gps_service_client(nest_gps_service_object)
	rospy.loginfo(str(result))
