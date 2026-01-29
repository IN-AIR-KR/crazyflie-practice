import torch
import numpy as np
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from crazyflie_interfaces.msg import FullState
from crazyflie_interfaces.srv import Takeoff

# ---------------------------------------------------------
# 1. MPPI Solver 클래스 (안정성 및 추진력 강화)
# ---------------------------------------------------------


class MPPISolver:
    def __init__(self, dt=0.02, T=100, K=800, lambda_=0.03):
        self.device = torch.device("cpu")
        self.dt = dt
        self.T = T
        self.K = K
        self.lambda_ = lambda_

        # 탐색 범위와 결단력 조절
        self.sigma = torch.tensor([1.5, 1.5], device=self.device)
        self.U = torch.zeros((self.T, 2), device=self.device)

    def solve(self, state, goal, obstacles):
        x0 = torch.tensor(state, dtype=torch.float32, device=self.device).repeat(self.K, 1)
        g = torch.tensor(goal, dtype=torch.float32, device=self.device)
        epsilon = torch.randn((self.K, self.T, 2), device=self.device) * self.sigma

        state_history = torch.zeros((self.K, self.T, 4), device=self.device)
        curr_states = x0.clone()

        # 미래 상태 예측 (Rollouts)
        for t in range(self.T):
            u_t = self.U[t] + epsilon[:, t, :]
            curr_states[:, :2] += curr_states[:, 2:] * self.dt
            curr_states[:, 2:] += u_t * self.dt
            state_history[:, t, :] = curr_states.clone()

        # 비용 계산
        costs = self._compute_costs(state_history, g, obstacles)

        # 가중 평균을 통한 최적 제어량 업데이트
        beta = torch.min(costs)
        weights = torch.exp(-1.0 / self.lambda_ * (costs - beta))
        weights /= (torch.sum(weights) + 1e-10)

        self.U += torch.sum(weights.view(self.K, 1, 1) * epsilon, dim=0)
        result_u = self.U[0].detach().numpy()
        self.U = torch.roll(self.U, -1, dims=0)
        self.U[-1] = 0.0

        return result_u, state_history.detach().numpy()

    def _compute_costs(self, state_history, goal, obstacles):
        # 1. 누적 비용 (Running Cost): 이동 중 목표와의 거리 합산
        all_dist = torch.norm(state_history[:, :, :2] - goal, dim=2)
        running_cost = torch.sum(all_dist, dim=1)

        # 2. 최종 비용 (Terminal Cost): 마지막 지점의 거리
        terminal_cost = torch.norm(state_history[:, -1, :2] - goal, dim=1)

        # 3. 장애물 비용
        obs_cost = torch.zeros(self.K, device=self.device)
        for obs in obstacles:
            dist = torch.norm(state_history[:, :, :2] - torch.tensor(obs[:2], device=self.device), dim=2)
            collision = (dist < (obs[2] + 0.3)).any(dim=1)  # 30cm 마진
            obs_cost += collision.float() * 10000.0
            obs_cost += torch.sum(torch.exp(-5.0 * (dist - obs[2])), dim=1)

        # 강력한 전진을 위한 가중치 합산
        return 15.0 * running_cost + 500.0 * terminal_cost + obs_cost

# ---------------------------------------------------------
# 2. ROS 2 제어 노드 (LPF 필터 및 상태 머신)
# ---------------------------------------------------------


class MppiStableNode(Node):
    def __init__(self):
        super().__init__("mppi_stable_node")

        # 웨이포인트 설정 (RRT* 연동을 위한 전역 경로 대용)
        self.waypoints = [[0.0, 2.0], [1.0, 3.5], [2.0, 4.0]]
        self.current_wp_idx = 0
        self.goal = self.waypoints[self.current_wp_idx]
        self.wp_threshold = 0.4

        # 제어 변수 및 필터 설정
        self.state = "IDLE"
        self.target_z = 0.5
        self.curr_state = [0.0, 0.0, 0.0, 0.0]
        self.alpha = 0.45  # 속도 필터 계수 (반응성 강화)
        self.last_pose_time = None

        self.obstacles = [[1.0, 2.0, 0.5]]
        self.mppi = MPPISolver()

        # 통신 인터페이스
        self.takeoff_client = self.create_client(Takeoff, "/cf1/takeoff")
        self.sub_pose = self.create_subscription(PoseStamped, "/cf1/pose", self._pose_cb, 10)
        self.pub_full_state = self.create_publisher(FullState, "/cf1/cmd_full_state", 10)
        self.pub_marker = self.create_publisher(MarkerArray, "/mppi_debug_markers", 10)

        self.create_timer(0.02, self._timer_cb)
        self.get_logger().info("✅ Integrated Stable MPPI Node Initialized")

    def _pose_cb(self, msg):
        t_now = self.get_clock().now()
        if self.last_pose_time is not None:
            # 시뮬레이션 클럭 기준 dt 계산 (추락 방지)
            dt = (t_now - self.last_pose_time).nanoseconds / 1e9
            if dt > 0.001:
                # 저역 통과 필터(LPF) 속도 추정
                raw_vx = (msg.pose.position.x - self.curr_state[0]) / dt
                raw_vy = (msg.pose.position.y - self.curr_state[1]) / dt
                self.curr_state[2] = self.alpha * raw_vx + (1 - self.alpha) * self.curr_state[2]
                self.curr_state[3] = self.alpha * raw_vy + (1 - self.alpha) * self.curr_state[3]

        self.curr_state[0] = msg.pose.position.x
        self.curr_state[1] = msg.pose.position.y
        self.last_pose_time = t_now

    def _timer_cb(self):
        if self.state == "IDLE":
            self._send_takeoff_request()
            self.state = "TAKEOFF"
            self.takeoff_start_time = time.time()
        elif self.state == "TAKEOFF":
            if time.time() - self.takeoff_start_time > 3.0:
                self.state = "MPPI"
        elif self.state == "MPPI":
            # 동적 웨이포인트 갱신 로직
            dist_to_wp = math.sqrt((self.curr_state[0]-self.goal[0])**2 + (self.curr_state[1]-self.goal[1])**2)
            if dist_to_wp < self.wp_threshold and self.current_wp_idx < len(self.waypoints)-1:
                self.current_wp_idx += 1
                self.goal = self.waypoints[self.current_wp_idx]
                self.get_logger().info(f"🚩 Heading to Waypoint {self.current_wp_idx}")

            u_opt, rollouts = self.mppi.solve(self.curr_state, self.goal, self.obstacles)
            self._publish_drone_cmd(u_opt)
            self._publish_visuals(rollouts)

    def _publish_drone_cmd(self, u):
        # 가속도 제한 완화 (기동성 강화)
        u_clamped = np.clip(u, -4.0, 4.0)

        msg = FullState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        dt = 0.02

        # 물리 방정식을 통한 다음 스텝 예측
        # $p_{next} = p_{curr} + v_{filt} \cdot dt + \frac{1}{2} a \cdot dt^2$
        next_vx = float(self.curr_state[2] + u_clamped[0] * dt)
        next_vy = float(self.curr_state[3] + u_clamped[1] * dt)
        next_x = float(self.curr_state[0] + next_vx * dt)
        next_y = float(self.curr_state[1] + next_vy * dt)

        msg.pose.position.x, msg.pose.position.y = next_x, next_y
        msg.pose.position.z = self.target_z
        msg.twist.linear.x, msg.twist.linear.y = next_vx, next_vy
        msg.acc.x, msg.acc.y = float(u_clamped[0]), float(u_clamped[1])

        yaw = math.atan2(next_vy, next_vx) if (next_vx**2 + next_vy**2) > 0.05 else 0.0
        msg.pose.orientation.z, msg.pose.orientation.w = float(math.sin(yaw*0.5)), float(math.cos(yaw*0.5))
        self.pub_full_state.publish(msg)

    def _send_takeoff_request(self):
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            pass
        req = Takeoff.Request()
        req.height, req.duration = self.target_z, rclpy.duration.Duration(seconds=2).to_msg()
        self.takeoff_client.call_async(req)
        self.get_logger().info("🛫 Automatic Takeoff Initiated")

    def _publish_visuals(self, rollouts):
        m_array = MarkerArray()
        obs_m = Marker(id=0, type=Marker.CYLINDER, action=Marker.ADD)
        obs_m.header.frame_id = "world"
        obs_m.pose.position.x, obs_m.pose.position.y, obs_m.pose.position.z = 1.0, 2.0, 0.25
        obs_m.scale.x = obs_m.scale.y = 1.0
        obs_m.scale.z = 0.5
        obs_m.color.r, obs_m.color.a = 1.0, 0.4
        m_array.markers.append(obs_m)

        roll_m = Marker(id=1, type=Marker.LINE_LIST, action=Marker.ADD)
        roll_m.header.frame_id = "world"
        roll_m.scale.x, roll_m.color.g, roll_m.color.a = 0.005, 1.0, 0.2
        for i in range(0, rollouts.shape[0], 25):
            for t in range(rollouts.shape[1]-1):
                roll_m.points.append(Point(x=float(rollouts[i, t, 0]), y=float(rollouts[i, t, 1]), z=0.5))
                roll_m.points.append(Point(x=float(rollouts[i, t+1, 0]), y=float(rollouts[i, t+1, 1]), z=0.5))
        m_array.markers.append(roll_m)
        self.pub_marker.publish(m_array)


def main():
    rclpy.init()
    node = MppiStableNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
