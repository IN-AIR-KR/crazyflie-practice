import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial import KDTree

# --- 설정 ---
NUM_DRONES = 50       # 드론 수 (애니메이션 성능을 위해 50~100대 추천)
COMM_RANGE = 15       # 통신 가능 거리
AREA_SIZE = 100       # 시뮬레이션 영역 (0 to 100)
SPEED = 1.5           # 드론 이동 속도 계수


class Drone:
    def __init__(self, id):
        self.id = id
        # 랜덤 위치 및 속도 벡터 초기화
        self.pos = np.random.rand(2) * AREA_SIZE
        self.vel = (np.random.rand(2) - 0.5) * SPEED

    def update_position(self):
        # 위치 업데이트
        self.pos += self.vel

        # 벽에 부딪히면 반사 (Simple Bouncing)
        for i in range(2):
            if self.pos[i] <= 0 or self.pos[i] >= AREA_SIZE:
                self.vel[i] *= -1
                self.pos[i] = np.clip(self.pos[i], 0, AREA_SIZE)


# 1. 드론 객체 리스트 생성
drones = [Drone(i) for i in range(NUM_DRONES)]

# 시각화 준비
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(0, AREA_SIZE)
ax.set_ylim(0, AREA_SIZE)
ax.set_title("Crazyflie Dynamic Mesh Simulation")

# 그래프 드로잉 요소 초기화
node_scatter = ax.scatter([], [], s=50, c='dodgerblue', edgecolors='white', zorder=3)
edge_lines = []  # 선 객체를 담을 리스트


def update(frame):
    global edge_lines
    # 이전 에지 라인들 제거
    for line in edge_lines:
        line.remove()
    edge_lines = []

    # 1. 모든 드론 위치 업데이트 및 추출
    current_pos = []
    for drone in drones:
        drone.update_position()
        current_pos.append(drone.pos)
    current_pos = np.array(current_pos)

    # 2. KD-Tree를 이용한 실시간 Mesh 구성
    tree = KDTree(current_pos)
    pairs = list(tree.query_pairs(COMM_RANGE))

    # 3. 네트워크 시각화 업데이트
    # 노드 위치 업데이트
    node_scatter.set_offsets(current_pos)

    # 에지(선) 그리기
    for i, j in pairs:
        line, = ax.plot([current_pos[i, 0], current_pos[j, 0]],
                        [current_pos[i, 1], current_pos[j, 1]],
                        color='gray', alpha=0.3, lw=1, zorder=1)
        edge_lines.append(line)

    return node_scatter, *edge_lines


# 애니메이션 실행
ani = FuncAnimation(fig, update, frames=200, interval=50, blit=True)
plt.show()
