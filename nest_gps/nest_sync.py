import rclpy
from rclpy.node import Node
from rclpy.client import Client
from msg_pkg.srv import UiMissionReq, ChrgDrone
from msg_pkg.msg import TelemMsg
from px4_msgs.msg import SensorGps, VehicleGlobalPosition
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile
import copy
import math

EARTH_RADIUS = 6371000 # in meters

class NestMissionNode(Node):
    def __init__(self):
        super().__init__('nest_mission_update')
        print("Initializing NestMissionNode...")
        self.qos_profile = QoSProfile(depth=10, reliability=0)
        # self.gps_subscription = self.create_subscription(
        #     NavSatFix, "nest_gps_info",
        #     self.gps_callback,self.qos_profile)
        self.telem_subscription = self.create_subscription(
            TelemMsg, "ui_telem_data",
            self.gps_callback, self.qos_profile
        )
        print("GPS subscription created.")
        self.last_lat = None
        self.last_lon = None
        self.gps_set = False
        self.sync_set = False
        self.sync_client = None
        self.sync_service = self.create_service(UiMissionReq,'mission_sync',self.start_sync,qos_profile = self.qos_profile)
        self.sync_client = self.create_client(UiMissionReq, 'ui_mission_upd',qos_profile = QoSProfile(depth=10,reliability=1))

        print("Sync service initialized.")

    def start_sync(self, request, response):
        print("Starting sync...")
        self.mission = request
        self.sync_set = True
        return response  # make sure to return a response

    # def gps_callback(self, msg):
    #     print("call back ongoing")
    #     if self.last_lat and self.last_lon and self.sync_set:
    #         distance = self.calculate_distance(self.last_lat, self.last_lon, msg.latitude, msg.longitude)
    #         self.get_logger().info(f'distance: {distance}')
    #         if distance > 5:
    #             print(f"Calculated distance: {distance}")
    #             self.get_logger().info('Calling service due to distance threshold breach.')
    #             self.call_service(msg)
    #             self.update_reference(msg)
    #     elif not self.gps_set:
    #         self.update_reference(msg)

    # def update_reference(self,msg):
    #     if msg.latitude and msg.longitude:
    #         self.last_lat = msg.latitude
    #         self.last_lon = msg.longitude
    #         self.gps_set = True
    #         print("GPS ref set")
    #         print(msg.latitude)
    #         print(msg.longitude)
    def gps_callback(self, msg):
        print("call back ongoing")
        if self.last_lat and self.last_lon and self.sync_set:
            distance = self.calculate_distance(self.last_lat, self.last_lon, msg.lat, msg.lon)  # Note the change here
            self.get_logger().info(f'distance: {distance}')
            if distance > 5:
                print(f"Calculated distance: {distance}")
                self.get_logger().info('Calling service due to distance threshold breach.')
                self.call_service(msg)
                self.update_reference(msg)
        elif not self.gps_set:
            self.update_reference(msg)

    def update_reference(self, msg):
        if msg.lat and msg.lon:  # Note the change here
            self.last_lat = msg.lat
            self.last_lon = msg.lon
            self.gps_set = True
            print("GPS ref set")
            print(msg.lat)
            print(msg.lon)
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat/2) * math.sin(dlat/2) + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2) * math.sin(dlon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = EARTH_RADIUS * c
        return distance

    def call_service(self, gps_data):
        try:
            # request = self.mission
            old_mission = copy.deepcopy(self.mission)
            request = UiMissionReq.Request()
            request.mission_type = old_mission.mission_type
            request.timestamp = gps_data.timestamp
            request.landing = old_mission.landing
            request.waypoints = old_mission.waypoints
            request.landing.position[0] = gps_data.lat
            request.landing.position[1] = gps_data.lon
            waypoint_set = False
            for waypoint in reversed(request.waypoints):
                if waypoint.position[0] != 0 and waypoint.position[1] != 0:  # Assuming 0,0 is the default empty filler
                    waypoint.position[0] = gps_data.lat
                    waypoint.position[1] = gps_data.lon
                    waypoint_set = True 
                    print(waypoint.position[0] , waypoint.position[1])
                    break
                else:
                    print(waypoint.position[0] , waypoint.position[1])
            if not waypoint_set:
                print("No non zero waypoint found to set")
            self.get_logger().info('Sending service request to '+self.mission.nest_id+'/ui_mission_upd')
            print(request)
            while not self.sync_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            # Asynchronously send the request and handle the response in a separate callback
            future = self.sync_client.call_async(request)
            future.add_done_callback(self.handle_send_mission_response)
        except Exception as e:
            self.get_logger().error(f"Exception in callback: {e}")

    def handle_send_mission_response(self, future):
        try:
            result = future.result()
            if result is not None:
                self.get_logger().info('Mission update sent successfully to drone')
                # Note: You might need to send or process the response here if necessary
            else:
                self.get_logger().error('Failed to send mission update')
        except Exception as e:
            self.get_logger().error(f"Exception in send mission response: {e}")

def main(args=None):
    rclpy.init(args=args)
    nest_mission_node = NestMissionNode()
    rclpy.spin(nest_mission_node)
    nest_mission_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
