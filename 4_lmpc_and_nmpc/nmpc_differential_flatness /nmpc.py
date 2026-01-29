import rclpy
from rclpy.node import Node
import numpy as np
import time
import math
from crazyflie_interfaces.msg import FullState
from crazyflie_interfaces.srv import Takeoff
from nav_msgs.msg import Odometry
import tf_transformations
from drone_model import setup_acados_solver


class NMPCFlatnessNode(Node):
    def __init__(self):
        super().__init__('nmpc_flatness_node')
        self.solver = setup_acados_solver()
        self.cmd_pub = self.create_publisher(FullState, '/cf1/cmd_full_state', 10)
        self.odom_sub = self.create_subscription(Odometry, '/cf1/odom', self.odom_cb, 10)
        self.takeoff_client = self.create_client(Takeoff, '/cf1/takeoff')

        # 8자 궤적 파라미터
        self.A, self.B, self.T = 0.3, 0.2, 4.0
        self.z_target = 0.5

        self.x_curr = np.zeros(7)
        self.state = "IDLE"
        self.start_time = None
        self.takeoff_done_time = None

        self.create_timer(0.01, self.control_loop)
        self.get_logger().info("NMPC Ready. Waiting for Odom to Takeoff...")

    def get_target_at(self, t):
        w = 2.0 * math.pi / self.T
        x = self.A * math.sin(w * t)
        y = self.B * math.sin(2.0 * w * t)
        vx = self.A * w * math.cos(w * t)
        vy = self.B * 2.0 * w * math.cos(2.0 * w * t)
        yaw = math.atan2(vy, vx) if (vx**2 + vy**2) > 1e-4 else 0.0
        return np.array([x, y, self.z_target, vx, vy, 0.0, yaw])

    def odom_cb(self, msg):
        p, v, q = msg.pose.pose.position, msg.twist.twist.linear, msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.x_curr = np.array([p.x, p.y, p.z, v.x, v.y, v.z, yaw])

        if self.state == "IDLE":
            self.get_logger().info("--- [1] Requesting Service Takeoff ---")
            req = Takeoff.Request()
            req.height = self.z_target
            req.duration = rclpy.duration.Duration(seconds=3.0).to_msg()
            self.takeoff_client.call_async(req)
            self.state = "TAKING_OFF"
            self.takeoff_start_time = time.time()

    def control_loop(self):
        if self.state == "TAKING_OFF":
            # 이륙 서비스 시작 후 4.5초 뒤에 NMPC로 전환
            if (time.time() - self.takeoff_start_time) > 4.5:
                self.state = "NMPC_ACTIVE"
                self.takeoff_done_time = time.time()
                self.get_logger().info("--- [2] NMPC Takeover! Figure-8 Start ---")
            return

        if self.state != "NMPC_ACTIVE":
            return

        t_flight = time.time() - self.takeoff_done_time
        dt_horizon = 0.015

        self.solver.set(0, "lbx", self.x_curr)
        self.solver.set(0, "ubx", self.x_curr)

        # 궤적 주입
        for i in range(20):
            t_future = t_flight + i * dt_horizon
            target = self.get_target_at(t_future)
            yref = np.concatenate([target, np.zeros(4)])
            self.solver.set(i, "yref", yref)

        self.solver.set(20, "yref", self.get_target_at(t_flight + 0.3))

        if self.solver.solve() == 0:
            nxt_u = self.solver.get(0, "u")
            target_now = self.get_target_at(t_flight)

            msg = FullState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"

            # Position/Velocity
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, target_now[:3])
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, target_now[3:6])

            # Orientation: NMPC 계산 각도 + 궤적 Yaw
            q = tf_transformations.quaternion_from_euler(float(nxt_u[0]), float(nxt_u[1]), float(target_now[6]))
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q

            msg.acc.z = float(np.clip(nxt_u[3] / 0.028, 9.0, 13.5))
            msg.twist.angular.z = float(nxt_u[2])

            self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = NMPCFlatnessNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
