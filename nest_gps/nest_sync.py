import rclpy
from rclpy.node import Node
from rclpy.client import Client
from msg_pkg.srv import UiMissionReq
from px4_msgs.msg import SensorGps
import math

EARTH_RADIUS = 6371000 # in meters

class ListenerNode(Node):

    def __init__(self):
        super().__init__('listener_node')
        self.declare_parameter('id', 'default_topic_name')
        topic_name = self.get_parameter('id').value
        self.subscription = self.create_subscription(
            SensorGps,
            topic_name+'/fmu/out/vehicle_gps_position',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.last_lat = None
        self.last_lon = None
        self.sync_client = self.create_client(UiMissionReq, topic_name+'/ui_mission_req_service')
        while not self.sync_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')

    def listener_callback(self, msg):
        if self.last_lat and self.last_lon:
            distance = self.calculate_distance(self.last_lat, self.last_lon, msg.lat, msg.lon)
            if distance > 5:
                self.call_service(msg)
                update_reference(msg)
        else:
            update_reference(msg)

    def update_reference(self,msg):
        self.last_lat = msg.lat
        self.last_lon = msg.lon

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat/2) * math.sin(dlat/2) + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2) * math.sin(dlon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = EARTH_RADIUS * c
        return distance

    def call_service(self, gps_data):
        # This function calls the service with the updated GPS data
        request = UiMissionReq.Request()
        request.timestamp = gps_data.timestamp
        request.landing.position[0] = gps_data.lat
        request.landing.position[1] = gps_data.lon
        future = self.sync_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            # Handle the response or log it
            self.get_logger().info('Received response: %r' % future.result())
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

def main(args=None):
    rclpy.init(args=args)
    listener_node = ListenerNode()
    rclpy.spin(listener_node)
    listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
