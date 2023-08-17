import rclpy
from rclpy.node import Node
import gps		# the gpsd interface module
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile
import time

class NestGPS(Node):

    def __init__(self):
        super().__init__('n_gps_pub')
        self.qos_profile = QoSProfile(depth=10, reliability=0)
        self.nest_gps_pub = self.create_publisher(NavSatFix,'nest_gps_info',self.qos_profile)
        timer_period = 2.0 #seconds
        self._last_update_time = time.time()
        self._average_latitude = 0.0
        self._average_longitude = 0.0
        self.msg_count = 0
        self.loop_cb()

    def loop_cb(self):
        session = gps.gps(mode=gps.WATCH_ENABLE)
        self.nest_gps_data = NavSatFix()

        try:
            while 0 == session.read():
                if not (gps.MODE_SET & session.valid):
                    # not useful, probably not a TPV message
                    continue

                # print('Mode: %s(%d) Time: ' %
                #     (("Invalid", "NO_FIX", "2D", "3D")[session.fix.mode],
                #     session.fix.mode), end="")
                # print time, if we have it
                if gps.TIME_SET & session.valid:
                    print(session.fix.time, end="")
                else:
                    print('n/a', end="")
                if ((gps.isfinite(session.fix.latitude) and
                    gps.isfinite(session.fix.longitude))):
                    if self.msg_count<10:
                        self.nest_gps_data.latitude = session.fix.latitude
                        self.nest_gps_data.longitude = session.fix.longitude
                        self.nest_gps_data.altitude = session.fix.altitude
                        self.msg_count += 1
                    else:
                        print(" Lat %.6f Lon %.6f" %
                            (session.fix.latitude, session.fix.longitude))
                        current_time = time.time()
                        dt = current_time - self._last_update_time
                        self._last_update_time = current_time
                        alpha = dt / (dt + 5.0)  # Filter coefficient using the calculated dt
                        # Apply low-pass filter for latitude and longitude
                        self._average_latitude = alpha * session.fix.latitude + (1 - alpha) * self._average_latitude
                        self._average_longitude = alpha * session.fix.longitude + (1 - alpha) * self._average_longitude
                        
                        self.nest_gps_data.latitude = self._average_latitude
                        self.nest_gps_data.longitude = self._average_longitude
                        self.nest_gps_data.altitude = session.fix.altitude
                    
                    self.nest_gps_pub.publish(self.nest_gps_data)
                else:
                    print(" Lat n/a Lon n/a")

        except KeyboardInterrupt:
            print('nest_gps interrupted')

        # Got ^C, or fell out of the loop.  Cleanup, and leave.
        session.close()
        exit(0)

def main(args=None):
    print("nest_gps node started")
    rclpy.init(args=args)
    nest_gps = NestGPS()
    rclpy.spin(nest_gps)
    nest_gps.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()