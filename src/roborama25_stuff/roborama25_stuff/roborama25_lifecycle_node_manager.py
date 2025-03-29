#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node, Client
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class Roborama25LifecycleNodeManager(Node):
    def __init__(self):
        super().__init__("roborama25_lifecycle_node_manager")
        self.declare_parameter("front_sensors_node_name", "sensors")
        self.declare_parameter("wheel_controller_node_name", "wheels")
        front_sensors_node_name = self.get_parameter("front_sensors_node_name").value
        wheels_controller_node_name = self.get_parameter("wheel_controller_node_name").value
        front_sensors_service_change_state_name = "/" + front_sensors_node_name + "/change_state"
        wheels_controller_service_change_state_name = "/" + wheels_controller_node_name + "/change_state"
        self.front_sensors_client = self.create_client(ChangeState, front_sensors_service_change_state_name)
        self.wheels_controller_client = self.create_client(ChangeState, wheels_controller_service_change_state_name)
        self.get_logger().info(f"""Initialized lifecycle_node_manager {front_sensors_service_change_state_name=} {wheels_controller_service_change_state_name=}""")
        
    def change_state(self, transition: Transition, client: Client):
        client.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
    
    def initialization_sequence(self):
        # Unconfigured to Inactive
        self.get_logger().info("Trying to switch to configuring")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        self.change_state(transition, self.front_sensors_client)
        self.change_state(transition, self.wheels_controller_client)
        self.get_logger().info("Configuring OK, now inactive")

        # sleep just for the example
        time.sleep(3)

        # Inactive to Active
        self.get_logger().info("Trying to switch to activating")
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"
        self.change_state(transition, self.front_sensors_client)
        self.change_state(transition, self.wheels_controller_client)
        self.get_logger().info("Activating OK, now active")


def main(args=None):
    rclpy.init(args=args)
    node = Roborama25LifecycleNodeManager()
    node.initialization_sequence()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
