#!/usr/bin/env python3
"""
3D Mesh Network Simulation: Flooding + Gossip (Anti-Entropy)
Crazyswarm2 / ROS2 버전 — RViz2 시각화

실제 드론 없이 순수 시뮬레이션으로 동작하며,
MarkerArray 토픽을 퍼블리시하여 RViz2에서 확인합니다.

실행:
  ros2 run <your_package> mesh_moving_flooding_gossip_ros2.py
  또는
  python3 mesh_moving_flooding_gossip_ros2.py

RViz2 설정:
  - Fixed Frame: world
  - Add > By topic > /mesh_network/markers (MarkerArray)
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import numpy as np
from scipy.spatial import KDTree
from collections import deque

# ---------------------------------------------------------------------------
# 시뮬레이션 설정
# ---------------------------------------------------------------------------
NUM_DRONES = 80          # 드론 수
COMM_RANGE = 1.5         # 통신 가능 거리 [m]
CUBE_X = 10.0            # 정육면체 x 크기 [m]
CUBE_Y = 10.0            # 정육면체 y 크기 [m]
CUBE_Z = 5.0             # 정육면체 z 크기 [m]
SPEED = 0.05             # 드론 속도 [m/스텝]
TIMER_PERIOD = 0.05      # 타이머 주기 [s] (20 Hz)
SEEN_PACKET_SIZE = 15    # Seen Packet Table 크기
INITIAL_TTL = NUM_DRONES # 패킷 초기 TTL 값
GOSSIP_INTERVAL = 5      # Gossip 실행 주기 (스텝 단위)
GOSSIP_PROB = 0.3        # Gossip 실행 확률 (각 이웃과)

# 드론 마커 크기
DRONE_SPHERE_RADIUS = 0.12   # [m]
EDGE_LINE_WIDTH = 0.02       # [m]


# ---------------------------------------------------------------------------
# Packet 헬퍼 함수
# ---------------------------------------------------------------------------

def create_packet(seq_num, sender_id, ttl, position):
    """패킷 생성 (구조체 형태의 dict)"""
    return {
        'seq_num': seq_num,
        'sender_id': sender_id,
        'ttl': ttl,
        'position': position.copy()   # 3D 좌표 (np.ndarray shape=(3,))
    }


def get_packet_id(packet):
    """패킷 고유 식별자 (sender_id, seq_num) 반환"""
    return (packet['sender_id'], packet['seq_num'])


# ---------------------------------------------------------------------------
# Drone 클래스 (3D)
# ---------------------------------------------------------------------------

class Drone:
    def __init__(self, drone_id, is_source=False):
        self.id = drone_id

        # 정육면체 내 랜덤 3D 위치 초기화
        self.pos = np.array([
            np.random.rand() * CUBE_X,
            np.random.rand() * CUBE_Y,
            np.random.rand() * CUBE_Z,
        ], dtype=float)

        # 랜덤 3D 속도 벡터 초기화
        self.vel = (np.random.rand(3) - 0.5) * SPEED

        self.has_message = is_source
        self.is_source = is_source

        # Seen Packet Table: 최근 받은 패킷 ID 저장 (중복 방지)
        self.seen_packets = deque(maxlen=SEEN_PACKET_SIZE)

        # 패킷 시퀀스 번호 (소스 드론만 사용)
        self.seq_num = 0

        # 전송 대기 버퍼
        self.packet_buffer = []

        # 전체 수신 패킷 저장소 (Gossip Digest 생성용)
        self.packet_storage = {}   # {(sender_id, seq_num): packet}

    # ------------------------------------------------------------------
    def update_position(self):
        """위치 업데이트 + 정육면체 경계 반사"""
        self.pos += self.vel

        limits = [CUBE_X, CUBE_Y, CUBE_Z]
        for i in range(3):
            if self.pos[i] <= 0.0:
                self.vel[i] *= -1.0
                self.pos[i] = 0.0
            elif self.pos[i] >= limits[i]:
                self.vel[i] *= -1.0
                self.pos[i] = limits[i]

    def reset_status(self):
        self.has_message = True if self.is_source else False
        self.seen_packets.clear()
        self.packet_buffer = []
        self.packet_storage.clear()

    # ------------------------------------------------------------------
    def create_packet_data(self):
        """새 패킷 생성 (소스 드론만 사용)"""
        packet = create_packet(self.seq_num, self.id, INITIAL_TTL, self.pos)
        self.seq_num += 1
        return packet

    def has_seen_packet(self, packet):
        return get_packet_id(packet) in self.seen_packets

    def receive_packet(self, packet):
        """패킷 수신 처리. 새로 받은 경우 True 반환."""
        if self.has_seen_packet(packet):
            return False
        if packet['ttl'] <= 0:
            return False

        packet_id = get_packet_id(packet)
        self.seen_packets.append(packet_id)
        self.has_message = True
        self.packet_storage[packet_id] = packet.copy()

        packet['ttl'] -= 1
        if packet['ttl'] > 0:
            self.packet_buffer.append(packet)

        return True

    def get_packets_to_forward(self):
        """전송할 패킷 리스트 반환 및 버퍼 비우기"""
        packets = self.packet_buffer.copy()
        self.packet_buffer = []
        return packets

    # ------------------------------------------------------------------
    # Gossip (Anti-Entropy)
    def create_digest(self):
        return set(self.packet_storage.keys())

    def get_missing_packets(self, peer_digest):
        return peer_digest - self.create_digest()

    def get_packets_by_ids(self, packet_ids):
        return [self.packet_storage[pid].copy()
                for pid in packet_ids if pid in self.packet_storage]


# ---------------------------------------------------------------------------
# ROS2 노드
# ---------------------------------------------------------------------------

class MeshNetworkSimNode(Node):

    def __init__(self):
        super().__init__('mesh_network_sim')

        self.publisher_ = self.create_publisher(
            MarkerArray, '/mesh_network/markers', 10)

        self.timer = self.create_timer(TIMER_PERIOD, self.simulation_step)

        np.random.seed(10)
        self.drones = [Drone(i, is_source=(i == 0)) for i in range(NUM_DRONES)]
        self.step_counter = 0

        # 첫 패킷 생성 플래그
        self._first_packet_sent = False

        self.get_logger().info(
            f'MeshNetworkSim started: {NUM_DRONES} drones, '
            f'cube={CUBE_X}x{CUBE_Y}x{CUBE_Z} m, '
            f'COMM_RANGE={COMM_RANGE} m'
        )

    # ------------------------------------------------------------------
    def simulation_step(self):
        self.step_counter += 1
        drones = self.drones

        # 1. 모든 드론 이동 및 위치 수집
        positions = np.array([d.pos for d in drones])
        for drone in drones:
            drone.update_position()
        positions = np.array([d.pos for d in drones])   # 이동 후 갱신

        # 2. KD-Tree로 통신 가능 쌍 탐색
        tree = KDTree(positions)
        all_pairs = list(tree.query_pairs(COMM_RANGE))

        # 3. 소스 드론이 첫 스텝에만 패킷 생성
        if self.step_counter == 1 and not self._first_packet_sent:
            for drone in drones:
                if drone.is_source:
                    packet = drone.create_packet_data()
                    drone.packet_buffer.append(packet)
                    drone.seen_packets.append(get_packet_id(packet))
                    self._first_packet_sent = True
                    break

        # 4. 패킷 전파 (Flooding)
        for idx, drone in enumerate(drones):
            packets_to_send = drone.get_packets_to_forward()
            if not packets_to_send:
                continue
            neighbors = tree.query_ball_point(positions[idx], COMM_RANGE)
            for n_idx in neighbors:
                if n_idx == idx:
                    continue
                for packet in packets_to_send:
                    drones[n_idx].receive_packet(packet)

        # 5. Gossip Anti-Entropy (주기적 동기화)
        if self.step_counter % GOSSIP_INTERVAL == 0:
            for idx, drone in enumerate(drones):
                neighbors = tree.query_ball_point(positions[idx], COMM_RANGE)
                for n_idx in neighbors:
                    if n_idx == idx:
                        continue
                    if np.random.rand() > GOSSIP_PROB:
                        continue
                    neighbor = drones[n_idx]
                    peer_digest = neighbor.create_digest()
                    missing_ids = drone.get_missing_packets(peer_digest)
                    if missing_ids:
                        for packet in neighbor.get_packets_by_ids(missing_ids):
                            packet_id = get_packet_id(packet)
                            if packet_id not in drone.packet_storage:
                                drone.packet_storage[packet_id] = packet.copy()
                                drone.seen_packets.append(packet_id)
                                drone.has_message = True

        # 6. 완료 확인 및 리셋
        infected_count = sum(d.has_message for d in drones)
        if infected_count == NUM_DRONES:
            self.get_logger().info(
                f'[Round Complete] 모든 드론 수신 완료! 소요 스텝: {self.step_counter}')
            self.step_counter = 0
            self._first_packet_sent = False
            for d in drones:
                d.reset_status()

        # 7. RViz2 MarkerArray 퍼블리시
        self._publish_markers(positions, all_pairs, infected_count)

    # ------------------------------------------------------------------
    def _publish_markers(self, positions, all_pairs, infected_count):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # --- 드론 구체 마커 (ID: 0 ~ NUM_DRONES-1) ---
        for i, drone in enumerate(self.drones):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = now
            m.ns = 'drones'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = float(positions[i, 0])
            m.pose.position.y = float(positions[i, 1])
            m.pose.position.z = float(positions[i, 2])
            m.pose.orientation.w = 1.0

            m.scale.x = DRONE_SPHERE_RADIUS * 2.0
            m.scale.y = DRONE_SPHERE_RADIUS * 2.0
            m.scale.z = DRONE_SPHERE_RADIUS * 2.0

            if drone.is_source:
                # 소스 드론: 빨간색
                m.color.r = 1.0
                m.color.g = 0.0
                m.color.b = 0.0
                m.color.a = 1.0
            elif drone.has_message:
                # 수신 완료: 주황색
                m.color.r = 1.0
                m.color.g = 0.5
                m.color.b = 0.0
                m.color.a = 1.0
            else:
                # 미수신: 파란색
                m.color.r = 0.1
                m.color.g = 0.4
                m.color.b = 1.0
                m.color.a = 1.0

            m.lifetime.sec = 0
            m.lifetime.nanosec = int(TIMER_PERIOD * 2 * 1e9)
            marker_array.markers.append(m)

        # --- 통신 에지 마커 (LINE_LIST, ID: NUM_DRONES) ---
        edge_marker = Marker()
        edge_marker.header.frame_id = 'world'
        edge_marker.header.stamp = now
        edge_marker.ns = 'edges'
        edge_marker.id = NUM_DRONES
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = EDGE_LINE_WIDTH
        edge_marker.color.r = 0.5
        edge_marker.color.g = 0.5
        edge_marker.color.b = 0.5
        edge_marker.color.a = 0.3
        edge_marker.lifetime.sec = 0
        edge_marker.lifetime.nanosec = int(TIMER_PERIOD * 2 * 1e9)

        for i, j in all_pairs:
            p1 = Point()
            p1.x = float(positions[i, 0])
            p1.y = float(positions[i, 1])
            p1.z = float(positions[i, 2])

            p2 = Point()
            p2.x = float(positions[j, 0])
            p2.y = float(positions[j, 1])
            p2.z = float(positions[j, 2])

            edge_marker.points.append(p1)
            edge_marker.points.append(p2)

        marker_array.markers.append(edge_marker)

        # --- 텍스트 상태 마커 (ID: NUM_DRONES + 1) ---
        text_marker = Marker()
        text_marker.header.frame_id = 'world'
        text_marker.header.stamp = now
        text_marker.ns = 'status'
        text_marker.id = NUM_DRONES + 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = CUBE_Z + 0.5
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.3
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = (
            f'Flooding + Gossip | Step: {self.step_counter} | '
            f'Infected: {infected_count}/{NUM_DRONES}'
        )
        text_marker.lifetime.sec = 0
        text_marker.lifetime.nanosec = int(TIMER_PERIOD * 2 * 1e9)
        marker_array.markers.append(text_marker)

        self.publisher_.publish(marker_array)


# ---------------------------------------------------------------------------
# 엔트리포인트
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MeshNetworkSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
