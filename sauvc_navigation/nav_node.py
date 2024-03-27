
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

from stingray_interfaces.action import MoveToObjectAction
from stingray_interfaces.msg import BboxArray
from stingray_core_interfaces.srv import SetTwist

from sauvc_navigation.map import Map


class NavNode(Node):

    def __init__(self):
        super().__init__("nav_node")
        self.declare_parameter("objects_array_topic",
                               "/stingray/topics/camera/objects")
        self.declare_parameter("set_twist_srv", "/stingray/services/set_twist")
        self.declare_parameter("move_to_object_action",
                               "/stingray/actions/move_to_object")

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

    def bbox_array_callback(self, bbox_array: BboxArray):
        self.get_logger().info(f"bbox_array: {bbox_array}")
        self.map.update(bbox_array)

    def execute_move_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("Executing goal...")

        twist_req = SetTwist.Request()

        while rclpy.ok():
            state, vector, img = self.map.calcPath(False, goal_handle.request.name)
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
