import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, EmptyResponse
import gps		# the gpsd interface module
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix
# from msg_pkg.srv import NestGPSMessage, NestGPSMessageResponse

class NestGPS(Node):

    def __init__(self):
        super().__init__('n_gps_pub')
        self.nest_gps_pub = self.create_publisher(NavSatFix,'/nest_gps_info',10)
        timer_period = 2.0 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)

    def timer_cb(self):
        rclpy.get_logger().info("/nest_gps_info topic started")
        session = gps.gps(mode=gps.WATCH_ENABLE)
        nest_gps_data = NavSatFix()
        # nest_gps_response = NestGPSMessageResponse()
        try:
            if 0 == session.read():
                if not (gps.MODE_SET & session.valid):
                    print('not gps.MODE_SET & session.valid')

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
                    self.nest_gps_pub.publish(nest_gps_data)
                else:
                    print(" Lat n/a Lon n/a Alt n/a")

        except KeyboardInterrupt:  # ^C
            print('/nest_gps service stopped')

        # session.close()

def main(args=None):
    print("nest_gps node started!")
    rclpy.init(args=args)
    nest_gps_pub = NestGPS()
    rclpy.spin(nest_gps_pub)
    nest_gps_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()