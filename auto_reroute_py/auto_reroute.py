import rclpy
from rclpy.node import Node
import carla
import random
import math

import yaml

from geometry_msgs.msg import PoseStamped
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import tf_transformations as tf

# Import custumed msg, srv from autoware ws
from autoware_auto_system_msgs.msg import AutowareState
from tier4_external_api_msgs.srv import Engage

class AutoReroute(Node):
    def __init__(self):
        super().__init__('auto_reroute')
        self.get_logger().info('Hi from auto_reroute_py.')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('is_random_route', rclpy.Parameter.Type.BOOL),
                ('carla_host', rclpy.Parameter.Type.STRING),
                ('carla_port', rclpy.Parameter.Type.INTEGER),
                ('waypoints_file', rclpy.Parameter.Type.STRING),
                ('min_distance', rclpy.Parameter.Type.DOUBLE),
            ]
        )

        # Get parameters set by launch file
        self._is_random_route = self.get_parameter('is_random_route').get_parameter_value().bool_value
        _carla_host = self.get_parameter('carla_host').get_parameter_value().string_value
        _carla_port = self.get_parameter('carla_port').get_parameter_value().integer_value
        _waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value
        _min_distance = self.get_parameter('min_distance').get_parameter_value().double_value

        # Define topic names needed
        _goal_topic = "/planning/mission_planning/goal"
        _autoware_state_topic = "/autoware/state"

        # Define srv name
        _engage_srv = "/api/external/set/engage"

        # if route is not random, get goal pose from parameter
        if not self._is_random_route:
            with open(_waypoints_file, 'r') as file:
                self._waypoints_data = yaml.safe_load(file)
            self.get_logger().info("waypoints_len: %f" % len(self._waypoints_data))
            self._waypoint_index = 0
        
        # Connect to CARLA
        self.client = carla.Client(_carla_host, _carla_port)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()
        self.spawn_points = self.world.get_map().get_spawn_points()

        # Initialize ROS publishers and subscribers

        # Create goal pose publisher
        self.goal_pub = self.create_publisher(
            PoseStamped,
            _goal_topic,
            1)

        # Subscribe to the autoware state to determine whether ego vehicle is waiting for engage
        self.create_subscription(
            AutowareState,
            _autoware_state_topic,
            self.autoware_state_callback,
            10)

        # Create TF listener to determine whether reroute is needed
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.tf_callback)

        # Create client for ego_vehicle engage
        self.engage_client = self.create_client(Engage, _engage_srv)

        # Define other variables
        self.current_goal_position = None
        self.reroute_threshold = _min_distance

    
    def tf_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            ego_vehicle_position = transform.transform.translation
            if self.current_goal_position is not None:
                distance = self.calculate_distance(ego_vehicle_position, self.current_goal_position)
                if distance < self.reroute_threshold:
                    self.publish_waypoints()
                    self.get_logger().info('Reroute...')
            else:
                self.publish_waypoints()
                self.get_logger().info('Sending first goal pose!')

        except Exception as e:
            # Handle any exceptions that may occur during transform lookup
            self.get_logger().error(f"Error: {str(e)}")
            
    def autoware_state_callback(self, msg):
        if (msg.state == AutowareState.WAITING_FOR_ENGAGE):
            self.get_logger().info('Sending engage request...')
            # Send engage request to ego vehicle
            engage_req = Engage.Request()
            engage_req.engage = True

            while not self.engage_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Engage service not available, waiting again...')

            future = self.engage_client.call_async(engage_req)
            
            # rclpy.spin_until_future_complete(self, future)
            # self.get_logger().info('Spin finihsed.')

            # if future.result() is not None:
            #     self.get_logger().info('Engage response: %d, %s' % (future.result().code, future.result().message))
            # else:
            #     self.get_logger().info('Service call failed %r' % (future.exception(),))

    def generate_waypoint(self):
        waypoint_data = self._waypoints_data[self._waypoint_index]
        goal_pose_position = waypoint_data["waypoint"]["position"]
        goal_pose_rpy = waypoint_data["waypoint"]["orientation_rpy"]

        self._waypoint_index = (self._waypoint_index + 1) % len(self._waypoints_data)

        return goal_pose_position, goal_pose_rpy

    def generate_waypoint_random(self):
        # Generate potential goal poses until valid
        sample = random.choice(self.spawn_points)
        goal_pose_position = [sample.location.x, sample.location.y, 0.0]

        # Loop through 0, pi/2, pi, 3pi/2 until valid
        # Transform from rpy to quaternion
        for i in range(4):
            goal_pose_rpy = [0, 0, i * math.pi / 2]
        
        return goal_pose_position, goal_pose_rpy
    
    def publish_waypoints(self):
        if self._is_random_route:
            waypoint_generator = self.generate_waypoint_random
        else:
            waypoint_generator = self.generate_waypoint
        
        goal_pose_position, goal_pose_rpy= waypoint_generator()
        goal_pose_quaternion = tf.quaternion_from_euler(goal_pose_rpy[0], goal_pose_rpy[1], goal_pose_rpy[2])

        # Initialize and set goal_pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'

        # Cast to float
        goal_pose.pose.position.x = float(goal_pose_position[0])
        goal_pose.pose.position.y = float(goal_pose_position[1])
        goal_pose.pose.position.z = float(goal_pose_position[2])
        goal_pose.pose.orientation.x = float(goal_pose_quaternion[0])
        goal_pose.pose.orientation.y = float(goal_pose_quaternion[1])
        goal_pose.pose.orientation.z = float(goal_pose_quaternion[2])
        goal_pose.pose.orientation.w = float(goal_pose_quaternion[3])

        self.current_goal_position = goal_pose.pose.position

        # Publish goal pose
        self.get_logger().info('Pose: %f, %f, %f' % (goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z))

        self.goal_pub.publish(goal_pose)
        self.get_logger().info('Published goal pose.')

    def calculate_distance(self, car_pos, goal_pos):
        # Calculate the Euclidean distance between two positions
        dx = car_pos.x - goal_pos.x
        dy = car_pos.y - goal_pos.y
        dz = car_pos.z - goal_pos.z
        distance = (dx ** 2 + dy ** 2 + dz ** 2) ** 0.5
        return distance
        

def main():
    rclpy.init()
    auto_reroute = AutoReroute()
    rclpy.spin(auto_reroute)
    auto_reroute.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
