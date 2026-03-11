import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial import KDTree
from collections import deque

# --- 시뮬레이션 설정 ---
NUM_DRONES = 80       # 드론 수
COMM_RANGE = 14      # 통신 가능 거리
AREA_SIZE = 100       # 시뮬레이션 영역 (0 to 100)
SPEED = 1.0           # 드론 속도 계수
INTERVAL = 50         # 애니메이션 프레임 속도 (ms)
SEEN_PACKET_SIZE = 15  # Seen Packet Table 크기
INITIAL_TTL = NUM_DRONES      # 패킷 초기 TTL 값
COMM_SUCCESS_RATE = 0.99  # 통신 성공 확률
GOSSIP_INTERVAL = 5    # Gossip 실행 주기 (스텝 단위)
GOSSIP_PROB = 0.3      # Gossip 실행 확률 (각 이웃과)

# --- Packet 구조체 (dict) 헬퍼 함수 ---


def create_packet(seq_num, sender_id, ttl, position):
    """패킷 생성 (구조체 형태의 dict)"""
    return {
        'seq_num': seq_num,      # 일련번호
        'sender_id': sender_id,  # 송신 드론 ID
        'ttl': ttl,              # Time To Live
        'position': position.copy()  # 패킷 생성 시점의 좌표
    }


def get_packet_id(packet):
    """패킷 고유 식별자 (sender_id, seq_num) 반환"""
    return (packet['sender_id'], packet['seq_num'])


# --- Drone 클래스 정의 ---


class Drone:
    def __init__(self, id, is_source=False):
        self.id = id
        # 영역 내 랜덤 위치 초기화
        self.pos = np.random.rand(2) * AREA_SIZE
        # 랜덤 속도 벡터 초기화
        self.vel = (np.random.rand(2) - 0.5) * SPEED
        # 메시지 보유 상태 (0번 드론만 True 시작)
        self.has_message = is_source
        self.is_source = is_source
        # Seen Packet Table: 최근 받은 패킷 ID 저장 (중복 방지)
        self.seen_packets = deque(maxlen=SEEN_PACKET_SIZE)
        # 패킷 시퀀스 번호 (소스 드론만 사용)
        self.seq_num = 0
        # 수신한 패킷 보관 (재전송용)
        self.packet_buffer = []
        # 패킷 저장소: 모든 수신 패킷 보관 (Gossip Digest 생성용)
        self.packet_storage = {}  # {(sender_id, seq_num): packet}

    def update_position(self):
        # 위치 업데이트
        self.pos += self.vel

        # 영역 경계 처리 (Simple Bouncing)
        for i in range(2):
            if self.pos[i] <= 0:
                self.vel[i] *= -1
                self.pos[i] = 0
            elif self.pos[i] >= AREA_SIZE:
                self.vel[i] *= -1
                self.pos[i] = AREA_SIZE

    def reset_status(self):
        # 상태 초기화
        self.has_message = True if self.is_source else False
        self.seen_packets.clear()
        self.packet_buffer = []
        self.packet_storage.clear()

    def create_packet_data(self):
        """새 패킷 생성 (소스 드론만 사용)"""
        packet = create_packet(self.seq_num, self.id, INITIAL_TTL, self.pos)
        self.seq_num += 1
        return packet

    def has_seen_packet(self, packet):
        """패킷을 이전에 받았는지 확인"""
        packet_id = get_packet_id(packet)
        return packet_id in self.seen_packets

    def receive_packet(self, packet):
        """패킷 수신 처리"""
        # 이미 본 패킷이면 무시
        if self.has_seen_packet(packet):
            return False

        # TTL이 0 이하면 무시
        if packet['ttl'] <= 0:
            return False

        # 새로운 패킷 수신
        packet_id = get_packet_id(packet)
        self.seen_packets.append(packet_id)
        self.has_message = True

        # 패킷 저장소에 보관 (Gossip용)
        self.packet_storage[packet_id] = packet.copy()

        # TTL 감소하여 버퍼에 저장 (재전송용)
        packet['ttl'] -= 1
        if packet['ttl'] > 0:
            self.packet_buffer.append(packet)

        return True

    def get_packets_to_forward(self):
        """전송할 패킷 리스트 반환 및 버퍼 비우기"""
        packets = self.packet_buffer.copy()
        self.packet_buffer = []
        return packets

    def create_digest(self):
        """Gossip Digest 생성: 현재 가진 패킷 ID 리스트 반환"""
        return set(self.packet_storage.keys())

    def get_missing_packets(self, peer_digest):
        """상대방이 가진 것 중 내가 없는 패킷 ID 리스트"""
        my_digest = self.create_digest()
        return peer_digest - my_digest

    def get_packets_by_ids(self, packet_ids):
        """특정 패킷 ID들에 해당하는 패킷 반환"""
        packets = []
        for pid in packet_ids:
            if pid in self.packet_storage:
                packets.append(self.packet_storage[pid].copy())
        return packets


# --- 초기화 ---
np.random.seed(10)  # 재현성을 위한 시드 (선택)
drones = [Drone(i, is_source=(i == 0)) for i in range(NUM_DRONES)]
step_counter = 0

# 시각화 설정
fig, ax = plt.subplots(figsize=(10, 9))
ax.set_xlim(0, AREA_SIZE)
ax.set_ylim(0, AREA_SIZE)

# 애니메이션 구성 요소
node_scatter = ax.scatter([], [], s=70, edgecolors='black', linewidth=0.5, zorder=3)
edge_collection = []  # 선 객체를 관리할 리스트

# --- 애니메이션 업데이트 함수 ---


def update(frame):
    global edge_collection, step_counter
    step_counter += 1

    # 1. 이전 에지(선) 지우기
    for line in edge_collection:
        line.remove()
    edge_collection = []

    # 2. 모든 드론 이동 및 위치 수집
    positions = []
    for drone in drones:
        drone.update_position()
        positions.append(drone.pos)
    positions = np.array(positions)

    # 3. 실시간 Mesh 네트워크 구성 (KD-Tree 사용)
    tree = KDTree(positions)
    # 현재 통신 가능한 모든 쌍 찾기
    all_pairs = list(tree.query_pairs(COMM_RANGE))

    # 4. 패킷 기반 메시지 전파 로직
    # 소스 드론이 새 패킷 생성
    for drone in drones:
        if drone.is_source and step_counter == 1:  # 첫 스텝에만 패킷 생성
            packet = drone.create_packet_data()
            drone.packet_buffer.append(packet)
            drone.seen_packets.append(get_packet_id(packet))
            break  # 소스 드론은 하나이므로 루프 종료

    # 모든 드론이 패킷 전파
    for idx, drone in enumerate(drones):
        packets_to_send = drone.get_packets_to_forward()
        # print(f"Drone {drone.id} has {len(packets_to_send)} packet(s) to forward.")
        if not packets_to_send:
            continue

        # 통신 범위 내 이웃 찾기
        neighbors = tree.query_ball_point(positions[idx], COMM_RANGE)
        # print(f"Drone {drone.id} has {len(packets_to_send)} packet(s) to send to neighbors: {neighbors}")
        # 각 이웃에게 패킷 전송
        for n_idx in neighbors:
            if n_idx == idx:  # 자기 자신 제외
                continue
            # print(f"Drone {drone.id} -> Drone {drones[n_idx].id} | Packets: {len(packets_to_send)}")
            for packet in packets_to_send:
                # if np.random.rand() < COMM_SUCCESS_RATE:  # 통신 성공 여부 결정
                #     drones[n_idx].receive_packet(packet)
                drones[n_idx].receive_packet(packet)

    # 4-2. Gossip 프로토콜 (Anti-Entropy): 주기적으로 이웃과 정보 동기화
    if step_counter % GOSSIP_INTERVAL == 0:
        for idx, drone in enumerate(drones):
            # 통신 범위 내 이웃 찾기
            neighbors = tree.query_ball_point(positions[idx], COMM_RANGE)

            # 랜덤하게 일부 이웃과만 Gossip (네트워크 부하 감소)
            for n_idx in neighbors:
                if n_idx == idx:  # 자기 자신 제외
                    continue

                # 확률적으로 Gossip 실행 (과도한 통신 방지)
                if np.random.rand() > GOSSIP_PROB:
                    continue

                neighbor = drones[n_idx]

                # Pull 모델: "넌 어떤 패킷들 갖고 있어?"
                # 1. Digest 교환
                my_digest = drone.create_digest()
                peer_digest = neighbor.create_digest()

                # 2. Difference 확인: 내가 없는 것
                missing_ids = drone.get_missing_packets(peer_digest)

                # 3. Update: 부족한 패킷만 요청해서 받기
                if missing_ids:
                    missing_packets = neighbor.get_packets_by_ids(missing_ids)
                    for packet in missing_packets:
                        # Gossip으로 받은 패킷은 TTL 감소 없이 직접 저장
                        packet_id = get_packet_id(packet)
                        if packet_id not in drone.packet_storage:
                            drone.packet_storage[packet_id] = packet.copy()
                            drone.seen_packets.append(packet_id)
                            drone.has_message = True

    # 5. 수신 완료 확인 및 리셋
    infected_count = sum(d.has_message for d in drones)
    if infected_count == NUM_DRONES:
        print(f"✅ [Round Complete] 모든 드론 수신 완료! 소요 시간: {step_counter} 스텝")
        step_counter = 0
        for d in drones:
            d.reset_status()

    # 6. 시각화 데이터 업데이트
    node_scatter.set_offsets(positions)

    # 색상 업데이트 (0번: Red, 수신: Orange, 미수신: Blue)
    colors = []
    for d in drones:
        if d.is_source:
            colors.append('red')
        elif d.has_message:
            colors.append('orange')
        else:
            colors.append('dodgerblue')
    node_scatter.set_color(colors)

    # 에지(연결선) 그리기
    for i, j in all_pairs:
        # 메시지를 전달할 수 있는 연결만 더 진하게 표시하는 등 커스텀 가능
        line, = ax.plot([positions[i, 0], positions[j, 0]],
                        [positions[i, 1], positions[j, 1]],
                        color='gray', alpha=0.15, lw=0.6, zorder=1)
        edge_collection.append(line)

    ax.set_title(
        f"Dynamic Drone Mesh: Flooding + Gossip (Anti-Entropy)\nSteps: {step_counter} | Infected: {infected_count}/{NUM_DRONES} | Gossip every {GOSSIP_INTERVAL} steps", fontsize=12)
    return node_scatter, *edge_collection


# --- 애니메이션 실행 ---
ani = FuncAnimation(fig, update, frames=None, interval=INTERVAL, blit=False)
plt.show()
