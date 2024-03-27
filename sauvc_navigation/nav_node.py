
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from stingray_interfaces.action import MoveToObjectAction
from stingray_interfaces.msg import BboxArray
from stingray_core_interfaces.srv import SetTwist
from stingray_core_interfaces.msg import UVState

from sauvc_navigation.map import Map
from sauvc_navigation.distance import DistanceCalculator


class NavNode(Node):

    def __init__(self):
        super().__init__("nav_node")
        self.declare_parameter("uv_state_topic",
                               "/stingray/topics/uv_state")
        self.declare_parameter("objects_array_topic",
                               "/stingray/topics/camera/objects")
        self.declare_parameter("set_twist_srv", "/stingray/services/set_twist")
        self.declare_parameter("move_to_object_action",
                               "/stingray/actions/move_to_object")

        self.declare_parameter("camera_fov", 60)
        self.declare_parameter("camera_resolution_x", 640)
        self.declare_parameter("camera_resolution_y", 480)

        self.imgsz = (self.get_parameter("camera_resolution_y").get_parameter_value(
        ).double_value, self.get_parameter("camera_resolution_x").get_parameter_value().double_value)
        self.fov = self.get_parameter("camera_fov").get_parameter_value().double_value

        self.dist_calc = DistanceCalculator(
            imgsz=self.imgsz,
            fov=self.fov,
            object_attrs={
                'gate': [1.5, 1.5, 4/5, 'blue'],
                'yellow_flare': [0.15*2, 1.5, 1.5/5, 'yellow'],
                'red_flare': [0.15*2, 1.5, 1.5/5, 'red'],
                'blue_bowl': [0.7, 0.35, 0.8/5, 'blue'],
                'red_bowl': [0.7, 0.35, 0.8/5, 'red']
            }
        )

        self.uv_state_sub = self.create_subscription(
            UVState,
            self.get_parameter(
                "uv_state_topic").get_parameter_value().string_value,
            self.uv_state_callback,
            1)

        self.bbox_array_sub = self.create_subscription(
            BboxArray,
            self.get_parameter(
                "objects_array_topic").get_parameter_value().string_value,
            self.bbox_array_callback,
            1)

        self.set_twist_client = self.create_client(
            SetTwist, self.get_parameter('set_twist_srv').get_parameter_value().string_value)

        while not self.set_twist_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f'set_twist_client not available, waiting again...')

        self.move_action_server = ActionServer(
            self,
            MoveToObjectAction,
            self.get_parameter(
                "move_to_object_action").get_parameter_value().string_value,
            self.execute_move_callback)

        self.i = 0
        self.map = Map([0, 0, 0], [0, 0, 90])

    def uv_state_callback(self, uv_state: UVState):
        self.get_logger().info(f"uv_state: {uv_state}")

    def bbox_array_callback(self, bbox_array: BboxArray):
        self.get_logger().info(f"bbox_array: {bbox_array}")
        self.map.update(bbox_array)

    def execute_move_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Executing goal...")

        twist_req = SetTwist.Request()

        while rclpy.ok():
            state, vector, img = self.map.calcPath(
                False, goal_handle.request.name)
            twist_req.surge = vector[5][0]
            twist_req.sway = vector[5][1]
            twist_req.yaw = vector[4][2]
            self.future = self.set_twist_client.call_async(twist_req)
            rclpy.spin_until_future_complete(self, self.future)

        goal_handle.succeed()

        result = MoveToObjectAction.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    nav_node = NavNode()

    rclpy.spin(nav_node)

    nav_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":

    main()
