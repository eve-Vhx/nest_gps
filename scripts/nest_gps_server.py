#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
import gps		# the gpsd interface module
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix
from msg_pkg.srv import NestGPSMessage, NestGPSMessageResponse


def my_callback(request):
	print("In the callback")
	rospy.loginfo("/nest_gps service started")
	session = gps.gps(mode=gps.WATCH_ENABLE)
	nest_gps_data = NavSatFix()
	# nest_gps_response = NestGPSMessageResponse()
	try:
		while 0 == session.read():
			if not (gps.MODE_SET & session.valid):
				# not useful, probably not a TPV message
				continue

			print('Mode: %s(%d) Time: ' %
				(("Invalid", "NO_FIX", "2D", "3D")[session.fix.mode],
					session.fix.mode), end="")
			# print time, if we have it
			if gps.TIME_SET & session.valid:
				print(session.fix.time, end="")
			else:
				print('n/a', end="")

			if ((gps.isfinite(session.fix.latitude) and
				gps.isfinite(session.fix.longitude))):
				print(" Lat %.6f Lon %.6f Alt %6f " %
					(session.fix.latitude, session.fix.longitude, session.fix.altitude))
				nest_gps_data.latitude = session.fix.latitude
				nest_gps_data.longitude = session.fix.longitude
				nest_gps_data.altitude = session.fix.altitude
				nest_gps_pub.publish(nest_gps_data)

				# nest_gps_response.latitude = session.fix.latitude
				# nest_gps_response.longitude = session.fix.longitude
				# nest_gps_response.altitude = session.fix.altitude
				
				# return nest_gps_response
				return NestGPSMessageResponse(session.fix.latitude,session.fix.longitude,session.fix.altitude)
			else:
				print(" Lat n/a Lon n/a Alt n/a")

	except KeyboardInterrupt:  # ^C
		print('/nest_gps service stopped')

	session.close()
	#exit(0)


if __name__ == '__main__':
	rospy.init_node('gps_server')
	nest_service = rospy.Service('/nest1_gps', NestGPSMessage, my_callback)
	nest_gps_pub = rospy.Publisher('/nest1_gps_info', NavSatFix, queue_size=10)
	rate = rospy.Rate(1)
	rate.sleep()
	print("Service started!")
	rospy.spin()