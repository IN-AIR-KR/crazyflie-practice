import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from scipy.spatial import KDTree


def run_drone_mesh_simulation(num_drones=100, comm_range=25, area_size=150):
    """
    num_drones: 드론 수 (수백 대까지 확장 가능)
    comm_range: 드론 간 통신 가능 거리 (m)
    area_size: 비행 구역 크기 (m x m)
    """

    # 1. 드론 초기 위치 생성 (Random Placement)
    # 실제 프로젝트에서는 Boids 알고리즘이나 고정 패턴을 넣을 수 있습니다.
    np.random.seed(42)  # 결과 재현을 위한 시드 고정
    positions = np.random.rand(num_drones, 2) * area_size
    pos_dict = {i: positions[i] for i in range(num_drones)}

    # 2. 그래프 객체 생성
    G = nx.Graph()
    G.add_nodes_from(range(num_drones))

    # 3. 효율적인 이웃 탐색 (KD-Tree 활용)
    # 모든 드론 쌍을 전수조사(N^2)하지 않고 통신 범위 내 노드만 빠르게 연결
    tree = KDTree(positions)
    pairs = tree.query_pairs(comm_range)
    G.add_edges_from(pairs)

    source_node = 5
    target_node = 85

    # 시각화 설정
    plt.figure(figsize=(10, 8))

    # 전체 네트워크 (기본 노드와 에지)
    nx.draw_networkx_nodes(G, pos_dict, node_size=200, node_color='lightgray', alpha=0.8)
    nx.draw_networkx_edges(G, pos_dict, edge_color='black', alpha=0.1)  # 기본 Mesh 링크는 흐리게
    nx.draw_networkx_labels(G, pos_dict, font_size=7, font_weight='bold')

    if target_node is not None:
        # 최단 경로 계산 (Dijkstra 기반)
        path = nx.shortest_path(G, source=source_node, target=target_node)
        path_edges = list(zip(path, path[1:]))

        # 경로 시각화 (빨간색 강조)
        nx.draw_networkx_nodes(G, pos_dict, nodelist=path, node_size=200, node_color='red')
        nx.draw_networkx_edges(G, pos_dict, edgelist=path_edges, edge_color='red', width=3)

        # 시작/종료 지점 별도 표시
        # plt.scatter(*positions[source_node], color='blue', s=200, label='Source (Drone 0)', zorder=5)
        # plt.scatter(*positions[target_node], color='green', s=200, label=f'Target (Drone {target_node})', zorder=5)

        plt.title(
            f"Drone Mesh Network Routing Simulation\nPath: {' -> '.join(map(str, path))} ({len(path)-1} Hops)", fontsize=14)
    else:
        plt.title("No Path Found (Network Disconnected)", fontsize=14)

    plt.legend(scatterpoints=1)
    plt.xlabel("X-axis (meters)")
    plt.ylabel("Y-axis (meters)")
    plt.grid(True, linestyle=':', alpha=0.5)
    plt.show()

    return G


# 시뮬레이션 실행
if __name__ == "__main__":
    mesh_graph = run_drone_mesh_simulation(num_drones=150, comm_range=20, area_size=120)
