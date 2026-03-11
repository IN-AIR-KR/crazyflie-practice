import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# 설정
NUM_DRONES = 100
COMM_RANGE = 20  # 통신 가능 거리
AREA_SIZE = 100

# 1. 드론 위치 랜덤 생성 (x, y)
positions = np.random.rand(NUM_DRONES, 2) * AREA_SIZE
print(positions)


def build_mesh(pos, dist_limit):
    G = nx.Graph()
    for i in range(NUM_DRONES):
        G.add_node(i, pos=pos[i])

    # KD-Tree로 근접 이웃 찾기 (효율적 연산)
    tree = KDTree(pos)
    pairs = tree.query_pairs(dist_limit)
    G.add_edges_from(pairs)
    return G


# 2. 메시 네트워크 생성
mesh_net = build_mesh(positions, COMM_RANGE)

# 3. 시각화
pos_dict = {i: positions[i] for i in range(NUM_DRONES)}
plt.figure(figsize=(8, 8))
nx.draw(mesh_net, pos_dict, node_size=30, node_color='blue', edge_color='gray', alpha=0.6)
plt.title(f"Drone Mesh Network (Nodes: {NUM_DRONES})")
plt.show()

# 4. 라우팅 시뮬레이션 (0번 드론에서 99번 드론으로 메시지 전달)
try:
    path = nx.shortest_path(mesh_net, source=0, target=99)
    print(f"Path found: {path}")
    print(f"Total Hops: {len(path) - 1}")
except nx.NetworkXNoPath:
    print("No path available between drones.")
