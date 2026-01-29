import rclpy
from rclpy.node import Node
import numpy as np
from crazyflie_interfaces.msg import FullState
from crazyflie_interfaces.srv import Takeoff
from nav_msgs.msg import Odometry
import tf_transformations
from drone_model import setup_acados_solver
import time


class CFHybrid7thNMPC(Node):
    def __init__(self):
        super().__init__('cf_7th_nmpc')
        self.solver = setup_acados_solver()
        self.g = 9.81
        self.x_curr = np.zeros(7)  # [px, py, pz, vx, vy, vz, yaw]
        self.target_state = None
        self.state = "IDLE"

        self.takeoff_client = self.create_client(Takeoff, '/cf1/takeoff')
        self.cmd_pub = self.create_publisher(FullState, '/cf1/cmd_full_state', 10)
        self.odom_sub = self.create_subscription(Odometry, '/cf1/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("7th-Order NMPC Online: Yaw Awareness Active")

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        q = msg.pose.pose.orientation
        # 쿼터니언에서 Yaw 추출
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.x_curr = np.array([p.x, p.y, p.z, v.x, v.y, v.z, yaw])

        if self.state == "IDLE":
            req = Takeoff.Request()
            req.height = 0.4
            req.duration = rclpy.duration.Duration(seconds=3.0).to_msg()
            self.takeoff_client.call_async(req)
            self.state = "TAKING_OFF"
            self.takeoff_start_time = time.time()

    def control_loop(self):
        if self.state == "TAKING_OFF":
            if (time.time() - self.takeoff_start_time) > 4.5:
                # 전환 시점의 위치와 Yaw를 타겟으로 고정
                self.target_state = np.array([self.x_curr[0], self.x_curr[1], self.x_curr[2], 0, 0, 0, self.x_curr[6]])
                self.state = "NMPC_ACTIVE"
                self.get_logger().info(f"NMPC Takeover! Locked Yaw: {self.target_state[6]:.2f}")
            return

        if self.state != "NMPC_ACTIVE":
            return

        self.solver.set(0, "lbx", self.x_curr)
        self.solver.set(0, "ubx", self.x_curr)
        yref = np.concatenate([self.target_state, np.zeros(4)])
        for i in range(20):
            self.solver.set(i, "yref", yref)
        self.solver.set(20, "yref", self.target_state)

        if self.solver.solve() == 0:
            nxt_x = self.solver.get(1, "x")
            nxt_u = self.solver.get(0, "u")

            # 가속도 벡터와 MPC 예측 Yaw를 결합한 자세 계산
            acc_total = np.array([nxt_u[0], nxt_u[1], nxt_u[2] + self.g])
            z_body = acc_total / np.linalg.norm(acc_total)

            # MPC가 계산한 다음 스텝의 Yaw 반영
            yaw_target = nxt_x[6]
            x_world_rot = np.array([np.cos(yaw_target), np.sin(yaw_target), 0.0])

            y_body = np.cross(z_body, x_world_rot)
            y_body /= np.linalg.norm(y_body)
            x_body = np.cross(y_body, z_body)
            R = np.column_stack([x_body, y_body, z_body])
            q = self.matrix_to_quat(R)

            msg = FullState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, self.target_state[:3])

            msg.acc.x, msg.acc.y = float(nxt_u[0]), float(nxt_u[1])
            msg.acc.z = float(np.clip(nxt_u[2] + self.g, 9.2, 10.5))

            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
            self.cmd_pub.publish(msg)

    def matrix_to_quat(self, R):
        tr = np.trace(R)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            return (R[2, 1]-R[1, 2])/S, (R[0, 2]-R[2, 0])/S, (R[1, 0]-R[0, 1])/S, 0.25*S
        return 0.0, 0.0, 0.0, 1.0


def main():
    rclpy.init()
    node = CFHybrid7thNMPC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
