import rclpy
from rclpy.node import Node
import numpy as np
from crazyflie_interfaces.msg import FullState
from crazyflie_interfaces.srv import Takeoff
from nav_msgs.msg import Odometry
import tf_transformations
from drone_model import setup_acados_solver
import time
import math


class NMPCRigorousNode(Node):
    def __init__(self):
        super().__init__('nmpc_rigorous_node')
        self.solver = setup_acados_solver()
        self.x_curr = np.zeros(7)
        self.target_state = None
        self.state = "IDLE"
        self.init_yaw = 0.0

        self.takeoff_client = self.create_client(Takeoff, '/cf1/takeoff')
        self.cmd_pub = self.create_publisher(FullState, '/cf1/cmd_full_state', 10)
        self.odom_sub = self.create_subscription(Odometry, '/cf1/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("Rigorous NMPC: Standing by for Odometry...")

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.x_curr = np.array([p.x, p.y, p.z, v.x, v.y, v.z, yaw])

        if self.state == "IDLE":
            self.init_yaw = yaw
            req = Takeoff.Request()
            req.height = 0.5
            req.duration = rclpy.duration.Duration(seconds=3.0).to_msg()
            self.takeoff_client.call_async(req)
            self.state = "TAKING_OFF"
            self.takeoff_start_time = time.time()
            self.get_logger().info("--- [1] Service Takeoff Start ---")

    def control_loop(self):
        if self.state == "TAKING_OFF":
            if (time.time() - self.takeoff_start_time) > 4.5:

                self.target_state = np.array([self.x_curr[0], self.x_curr[1], 0.5, 0, 0, 0, self.init_yaw])
                self.state = "NMPC_ACTIVE"
                self.get_logger().info(f"--- [2] NMPC Takeover! Locked at {self.target_state[2]}m ---")
            return

        if self.state != "NMPC_ACTIVE":
            return

        # NMPC 연산
        self.solver.set(0, "lbx", self.x_curr)
        self.solver.set(0, "ubx", self.x_curr)
        yref = np.concatenate([self.target_state, np.zeros(4)])
        for i in range(20):
            self.solver.set(i, "yref", yref)
        self.solver.set(20, "yref", self.target_state)

        if self.solver.solve() == 0:
            nxt_u = self.solver.get(0, "u")  # [roll, pitch, yaw_dot, thrust]

            msg = FullState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"

            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, self.target_state[:3])

            q = tf_transformations.quaternion_from_euler(float(nxt_u[0]), float(nxt_u[1]), float(self.init_yaw))
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q

            msg.acc.z = float(np.clip(nxt_u[3] / 0.028, 9.0, 15.0))
            msg.twist.angular.z = float(nxt_u[2])

            self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = NMPCRigorousNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
