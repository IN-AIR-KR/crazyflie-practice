import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


class CBAA:
    def __init__(self, num_drones, agent_id):
        self.num_drones = num_drones
        self.agent_id = agent_id
        self.bids = np.zeros(num_drones)
        self.winners = np.full(num_drones, -1, dtype=int)  # 각 타겟의 낙찰자 ID
        self.target_id = None
        self.my_bids = np.zeros(num_drones)  # 내가 계산한 입찰가 (시각화용)

    def update(self, agent_pos, target_pos_list, neighbors_info):
        self.auction_process(agent_pos, target_pos_list)
        self.consensus_process(neighbors_info)
        return self.target_id

    def auction_process(self, agent_pos, target_pos_list):
        # 나의 입찰가 계산 (순수 거리 기반) - 항상 계산하여 시각화에 사용
        my_bids = np.zeros(self.num_drones)
        for i in range(self.num_drones):
            distance = np.linalg.norm(agent_pos - target_pos_list[i])
            bid = 1 / (distance + 1e-6)
            my_bids[i] = bid

        self.my_bids = my_bids  # 시각화를 위해 저장

        if self.target_id is not None:
            return

        # 도전 가능한 임무만 선택
        target_candidate = np.zeros(self.num_drones, dtype=bool)
        for i in range(self.num_drones):
            # 내 입찰가가 더 높거나, 동점이면 agent_id가 더 높을 때 도전 가능
            if my_bids[i] > self.bids[i]:
                target_candidate[i] = True
            elif np.isclose(my_bids[i], self.bids[i]) and self.agent_id > self.winners[i]:
                target_candidate[i] = True

        if np.any(target_candidate):
            # 도전 가능한 임무 중에서 가장 높은 입찰가를 가진 임무 선택
            masked_bids = np.where(target_candidate, my_bids, -np.inf)
            self.target_id = np.argmax(masked_bids)
            self.bids[self.target_id] = my_bids[self.target_id]
            self.winners[self.target_id] = self.agent_id
            print(f"Drone {self.agent_id} bids for target {self.target_id} with bid {self.bids[self.target_id]:.4f}")
        elif self.target_id is None:
            # 도전 가능한 임무가 없고 현재 타겟도 없으면, 가장 가까운 임무라도 선택
            self.target_id = np.argmax(my_bids)
            self.bids[self.target_id] = my_bids[self.target_id]
            self.winners[self.target_id] = self.agent_id

    def consensus_process(self, neighbors_info):
        # 각 이웃의 입찰가 및 낙찰자 정보와 비교하여 업데이트
        for neighbor_bids, neighbor_winners in neighbors_info:
            for i in range(self.num_drones):
                # 이웃의 입찰가가 더 높으면 업데이트
                if neighbor_bids[i] > self.bids[i]:
                    self.bids[i] = neighbor_bids[i]
                    self.winners[i] = neighbor_winners[i]
                # 동점이면 agent_id가 높은 쪽의 정보로 업데이트
                elif np.isclose(neighbor_bids[i], self.bids[i]) and neighbor_winners[i] > self.winners[i]:
                    self.bids[i] = neighbor_bids[i]
                    self.winners[i] = neighbor_winners[i]

        # 충돌 해결: 내가 선택한 타겟의 낙찰자가 나가 아니면 포기
        if self.target_id is not None:
            if self.winners[self.target_id] != self.agent_id:
                print(
                    f"Drone {self.agent_id} releases target {self.target_id} (winner: {self.winners[self.target_id]})")
                self.target_id = None


class Drone:
    """드론 클래스 - 현재 위치와 목표 위치를 가지고 있음"""

    def __init__(self, drone_id, start_pos, target_positions, num_drones):
        self.id = drone_id
        self.pos = np.array(start_pos, dtype=float)
        self.target_positions = target_positions
        self.target = np.array(start_pos, dtype=float)
        self.speed = 0.05  # 이동 속도
        self.reached = False
        self.cbaa = CBAA(num_drones, drone_id)  # agent_id 전달

    def update(self, neighbors_info):
        """목표 지점으로 이동"""
        if not self.reached and self.target is not None:
            # 목표까지의 거리
            direction = self.target - self.pos
            distance = np.linalg.norm(direction)

            if distance < 0.05:  # 충분히 가까우면 도착
                self.pos = self.target
                self.reached = True
            else:
                # 정규화된 방향으로 이동
                self.pos += (direction / distance) * self.speed

        # CBAA 업데이트
        target_id = self.cbaa.update(self.pos, self.target_positions, neighbors_info)
        if target_id is not None:
            self.set_target(self.target_positions[target_id])

    def get_position(self):
        """현재 위치 반환"""
        return self.pos.copy()

    def set_target(self, target_pos):
        """목표 위치 설정"""
        self.target = np.array(target_pos, dtype=float)
        self.reached = False


class DroneFormation:
    """드론 편대 시뮬레이션"""

    def __init__(self):
        # 초기 위치: 1열로 정렬 (y=0 라인에 x축으로 배치)
        self.start_positions = [
            [0.0, 0.0],   # 왼쪽 드론
            [1.0, 0.0],   # 중앙 드론
            [2.3, 0.0]    # 오른쪽 드론
        ]

        # 목표 위치: V자 형태
        self.target_positions = [
            [0.5, 0.5],   # 왼쪽 위
            [1.0, 1.0],   # 중앙 (V의 꼭지점)
            [2.0, 2.0]    # 오른쪽 위
        ]

        # 드론 생성
        self.drones = [
            Drone(i, self.start_positions[i], self.target_positions, len(self.start_positions))
            for i in range(3)
        ]

        # 시각화 설정 - 메인 그래프 + 드론별 입찰가 막대 그래프
        self.fig = plt.figure(figsize=(16, 10))
        gs = self.fig.add_gridspec(3, 2, width_ratios=[2, 1], hspace=0.5, wspace=0.3)

        # 메인 드론 위치 그래프 (왼쪽, 전체 높이)
        self.ax = self.fig.add_subplot(gs[:, 0])

        # 각 드론별 입찰가 막대 그래프 (오른쪽, 3개)
        self.bid_axes = [
            self.fig.add_subplot(gs[i, 1]) for i in range(3)
        ]

        self.anim = None  # 애니메이션 객체 저장용
        self.setup_plot()

    def setup_plot(self):
        """플롯 초기 설정"""
        self.ax.set_xlim(-0.5, 2.5)
        self.ax.set_ylim(-0.5, 2.5)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X Position (m)', fontsize=12)
        self.ax.set_ylabel('Y Position (m)', fontsize=12)
        self.ax.set_title('Drone Formation Test', fontsize=14, fontweight='bold')

        # 드론별 색상
        self.colors = ['red', 'blue', 'green']

        # 초기 위치와 목표 위치 표시
        for i, target in enumerate(self.target_positions):
            # 목표 위치 (큰 X 마커)
            self.ax.plot(target[0], target[1], 'x',
                         color='gray', markersize=15,
                         markeredgewidth=3, alpha=0.5,
                         label=f'Drone {i+1} Target')
            # 목표 위치 텍스트 레이블
            self.ax.text(target[0] + 0.1, target[1] + 0.1, f'T{i}',
                         fontsize=12, fontweight='bold', color='black',
                         bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))

        # 드론 현재 위치 (업데이트될 scatter plot)
        initial_positions = np.array([drone.get_position() for drone in self.drones])
        self.scatter = self.ax.scatter(initial_positions[:, 0], initial_positions[:, 1],
                                       s=200, c=self.colors,
                                       marker='o', edgecolors='black',
                                       linewidths=2, zorder=5)

        # 궤적을 저장할 리스트
        self.trajectories = [[] for _ in range(len(self.drones))]
        self.trajectory_lines = [
            self.ax.plot([], [], '--', color=self.colors[i], alpha=0.6, linewidth=1)[0]
            for i in range(len(self.drones))
        ]

        # 입찰가 막대 그래프 초기화
        self.bid_bars = []
        for i, ax_bid in enumerate(self.bid_axes):
            ax_bid.set_title(f'Drone {i} Bids', fontsize=10, fontweight='bold',
                             color=self.colors[i], pad=8)
            ax_bid.set_xlabel('Target ID', fontsize=9)
            ax_bid.set_ylabel('Bid Value', fontsize=9)
            ax_bid.set_ylim(0, 2.5)
            ax_bid.grid(True, alpha=0.3, axis='y')
            ax_bid.tick_params(axis='both', labelsize=8)

            # 초기 막대 그래프 생성
            x_pos = np.arange(len(self.drones))
            bars = ax_bid.bar(x_pos, np.zeros(len(self.drones)),
                              color='lightgray', edgecolor='black', linewidth=1, width=0.6)
            self.bid_bars.append(bars)

            ax_bid.set_xticks(x_pos)
            ax_bid.set_xticklabels([f'T{j}' for j in range(len(self.drones))], fontsize=8)

    def update_frame(self, frame):
        """애니메이션 프레임 업데이트"""

        # 모든 드론의 입찰가 및 낙찰자 정보 수집
        cbaa_info = [(drone.cbaa.bids, drone.cbaa.winners) for drone in self.drones]

        # 모든 드론 업데이트
        for i, drone in enumerate(self.drones):
            neighbors_info = cbaa_info[:i] + cbaa_info[i+1:]
            drone.update(neighbors_info)

        # 현재 위치 수집
        positions = np.array([drone.get_position() for drone in self.drones])

        # 궤적 저장
        for i, pos in enumerate(positions):
            self.trajectories[i].append(pos.copy())
            if len(self.trajectories[i]) > 1:
                traj = np.array(self.trajectories[i])
                self.trajectory_lines[i].set_data(traj[:, 0], traj[:, 1])

        # Scatter plot 업데이트
        self.scatter.set_offsets(positions)

        # 입찰가 막대 그래프 업데이트
        for i, drone in enumerate(self.drones):
            my_bids = drone.cbaa.my_bids
            target_id = drone.cbaa.target_id

            for j, bar in enumerate(self.bid_bars[i]):
                bar.set_height(my_bids[j])
                # 선택된 타겟은 해당 드론 색상으로, 나머지는 회색으로
                if j == target_id:
                    bar.set_color(self.colors[i])
                    bar.set_alpha(0.9)
                else:
                    bar.set_color('lightgray')
                    bar.set_alpha(0.6)

        # 모든 드론이 도착했는지 확인
        all_reached = all(drone.reached for drone in self.drones)
        if all_reached and frame > 10:
            print("모든 드론이 목표 지점에 도착했습니다!")
            if self.anim:
                self.anim.event_source.stop()  # 애니메이션 종료

        return self.scatter, *self.trajectory_lines

    def run(self, save_gif=False, gif_filename='cbaa_simulation.gif'):
        """시뮬레이션 실행"""
        self.anim = FuncAnimation(self.fig, self.update_frame,
                                  frames=40, interval=100,
                                  blit=False, repeat=False)  # blit=False로 변경 (막대 그래프 업데이트)

        if save_gif:
            print(f"GIF 저장 중: {gif_filename} (시간이 걸릴 수 있습니다...)")
            writer = PillowWriter(fps=10)
            self.anim.save(gif_filename, writer=writer)
            print(f"GIF 저장 완료: {gif_filename}")
        else:
            plt.show()


if __name__ == "__main__":
    print("드론 편대 변경 시뮬레이션 시작")

    formation = DroneFormation()
    formation.run(save_gif=True)
    # formation.run(save_gif=False)
