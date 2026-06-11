import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class SpotRobotManager(Node):
    def __init__(self):
        super().__init__("spot_robot_manager")

        self.declare_parameter("robot_name", "")
        self.declare_parameter("auto_claim", True)
        self.declare_parameter("auto_power_on", True)
        self.declare_parameter("auto_stand", True)
        self.declare_parameter("stop_on_shutdown", True)
        self.declare_parameter("sit_on_shutdown", False)
        self.declare_parameter("service_timeout_sec", 10.0)
        self.declare_parameter("stand_settle_sec", 3.0)

        self.robot_name = self.get_parameter("robot_name").value
        self.auto_claim = bool(self.get_parameter("auto_claim").value)
        self.auto_power_on = bool(self.get_parameter("auto_power_on").value)
        self.auto_stand = bool(self.get_parameter("auto_stand").value)
        self.stop_on_shutdown = bool(self.get_parameter("stop_on_shutdown").value)
        self.sit_on_shutdown = bool(self.get_parameter("sit_on_shutdown").value)
        self.service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        self.stand_settle_sec = float(self.get_parameter("stand_settle_sec").value)

        self.clients = {
            "claim": self.create_client(Trigger, self.namespaced("claim")),
            "power_on": self.create_client(Trigger, self.namespaced("power_on")),
            "stand": self.create_client(Trigger, self.namespaced("stand")),
            "stop": self.create_client(Trigger, self.namespaced("stop")),
            "sit": self.create_client(Trigger, self.namespaced("sit")),
            "release": self.create_client(Trigger, self.namespaced("release")),
        }

        self.ready = False

    def namespaced(self, service_name):
        if self.robot_name:
            return f"/{self.robot_name.strip('/')}/{service_name}"
        return f"/{service_name}"

    def initialize_robot(self):
        sequence = []
        if self.auto_claim:
            sequence.append("claim")
        if self.auto_power_on:
            sequence.append("power_on")
        if self.auto_stand:
            sequence.append("stand")

        for command in sequence:
            if not self.call_trigger(command):
                self.get_logger().error(f"Spot initialization stopped at '{command}'")
                return False
            if command == "stand":
                time.sleep(self.stand_settle_sec)

        self.ready = True
        self.get_logger().info("Spot is initialized for human following")
        return True

    def shutdown_robot(self):
        if self.stop_on_shutdown:
            self.call_trigger("stop", required=False)
        if self.sit_on_shutdown:
            self.call_trigger("sit", required=False)
        if self.auto_claim:
            self.call_trigger("release", required=False)

    def call_trigger(self, command, required=True):
        client = self.clients[command]
        service_name = self.namespaced(command)
        self.get_logger().info(f"Waiting for {service_name}")
        if not client.wait_for_service(timeout_sec=self.service_timeout_sec):
            message = f"Service {service_name} is not available"
            if required:
                self.get_logger().error(message)
            else:
                self.get_logger().warning(message)
            return False

        self.get_logger().info(f"Calling {service_name}")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout_sec)

        if not future.done():
            self.get_logger().error(f"Service call to {service_name} timed out")
            return False

        response = future.result()
        if response is None:
            self.get_logger().error(f"Service call to {service_name} failed")
            return False

        if not response.success:
            log = self.get_logger().error if required else self.get_logger().warning
            log(f"{service_name} returned failure: {response.message}")
            return False

        self.get_logger().info(f"{service_name} succeeded: {response.message}")
        return True


def main(args=None):
    rclpy.init(args=args)
    manager = SpotRobotManager()
    try:
        if manager.initialize_robot():
            rclpy.spin(manager)
    finally:
        manager.shutdown_robot()
        manager.destroy_node()
        rclpy.shutdown()
