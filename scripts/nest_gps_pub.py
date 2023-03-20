#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
import gps		# the gpsd interface module
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix
from msg_pkg.srv import NestGPSMessage, NestGPSMessageResponse

class NestGPSPub:

	def __init__(self):
		self.nest_gps_pub = rospy.Publisher('nest_gps_pub', NavSatFix, queue_size=10)
		self.session = gps.gps(mode=gps.WATCH_ENABLE)
		self.run_gps_routine()

	def run_gps_routine(self):
		while 0 == self.session.read():
			nest_gps_data = NavSatFix()
			print('Mode: %s(%d) Time: ' %
				(("Invalid", "NO_FIX", "2D", "3D")[self.session.fix.mode],
					self.session.fix.mode), end="")

			if gps.TIME_SET & self.session.valid:
				print(self.session.fix.time, end="")
			else:
				print('n/a', end="")

			if ((gps.isfinite(self.session.fix.latitude) and
				gps.isfinite(self.session.fix.longitude))):
				print(" Lat %.6f Lon %.6f Alt %6f " %
					(self.session.fix.latitude, self.session.fix.longitude, self.session.fix.altitude))
				nest_gps_data.latitude = self.session.fix.latitude
				nest_gps_data.longitude = self.session.fix.longitude
				nest_gps_data.altitude = self.session.fix.altitude

				self.nest_gps_pub.publish(nest_gps_data)

			else:
				print(" Lat n/a Lon n/a Alt n/a")

			rospy.sleep(2)

		self.session.close()


if __name__ == '__main__':
	rospy.init_node('gps_publisher')
	NestGPSPub()
	rospy.spin()