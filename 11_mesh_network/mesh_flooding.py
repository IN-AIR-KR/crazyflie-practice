import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial import KDTree

# --- 설정 ---
NUM_DRONES = 80       # 드론 수
COMM_RANGE = 20       # 통신 가능 거리
AREA_SIZE = 100
SPEED = 1.2


class Drone:
    def __init__(self, id):
        self.id = id
        self.pos = np.random.rand(2) * AREA_SIZE
        self.vel = (np.random.rand(2) - 0.5) * SPEED
        self.has_message = True if id == 0 else False  # 0번 드론만 메시지 보유 시작

    def update_position(self):
        self.pos += self.vel
        # 벽 반사 로직
        for i in range(2):
            if self.pos[i] <= 0 or self.pos[i] >= AREA_SIZE:
                self.vel[i] *= -1
                self.pos[i] = np.clip(self.pos[i], 0, AREA_SIZE)

    def reset_message(self):
        self.has_message = True if self.id == 0 else False


# 1. 드론 객체 생성
drones = [Drone(i) for i in range(NUM_DRONES)]
steps_taken = 0

# 시각화 설정
fig, ax = plt.subplots(figsize=(9, 8))
ax.set_xlim(0, AREA_SIZE)
ax.set_ylim(0, AREA_SIZE)
node_scatter = ax.scatter([], [], s=60, edgecolors='black', zorder=3)
edge_lines = []


def update(frame):
    global edge_lines, steps_taken
    steps_taken += 1

    # 이전 라인 제거
    for line in edge_lines:
        line.remove()
    edge_lines = []

    # 위치 업데이트 및 데이터 추출
    current_pos = np.array([d.pos for d in drones])
    has_msg_array = np.array([d.has_message for d in drones])

    # 2. 메시지 전파 로직 (Gossip)
    tree = KDTree(current_pos)
    # 메시지를 가진 드론들의 인덱스 추출
    infected_indices = np.where(has_msg_array == True)[0]

    # 메시지를 가진 드론 주변의 모든 이웃 찾기
    for idx in infected_indices:
        neighbors = tree.query_ball_point(current_pos[idx], COMM_RANGE)
        for n_idx in neighbors:
            drones[n_idx].has_message = True

    # 3. 모든 드론 수신 확인 및 리셋
    if all(d.has_message for d in drones):
        print(f"✅ 모든 드론 수신 완료! 소요 시간: {steps_taken} steps")
        steps_taken = 0
        for d in drones:
            d.reset_message()

    # 4. 시각화 업데이트
    # 색상 결정: 0번은 Red, 수신자는 Orange, 미수신자는 Blue
    colors = []
    for d in drones:
        if d.id == 0:
            colors.append('red')
        elif d.has_message:
            colors.append('orange')
        else:
            colors.append('dodgerblue')

    node_scatter.set_offsets(current_pos)
    node_scatter.set_color(colors)

    # 연결선 표시 (선택 사항: 메시지 보유 노드 간의 연결만 표시하거나 전체 표시)
    pairs = list(tree.query_pairs(COMM_RANGE))
    for i, j in pairs:
        line, = ax.plot([current_pos[i, 0], current_pos[j, 0]],
                        [current_pos[i, 1], current_pos[j, 1]],
                        color='gray', alpha=0.15, lw=0.5, zorder=1)
        edge_lines.append(line)

    ax.set_title(f"Drone Mesh Gossip Simulation | Steps: {steps_taken}")
    return node_scatter, *edge_lines


ani = FuncAnimation(fig, update, frames=None, interval=50, blit=False, cache_frame_data=False)
plt.show()
