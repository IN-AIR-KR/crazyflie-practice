import numpy as np
from crazyflie_py import Crazyswarm
import time


class CBBAAgent:
    def __init__(self, agent_id, num_tasks):
        self.id = agent_id
        self.num_tasks = num_tasks
        self.bundle = []
        self.winning_bids = np.zeros(num_tasks)
        self.winners = np.full(num_tasks, -1)

    def calculate_score(self, task_id, task_pos, current_pos):
        # PDF p.6-7: 보상과 거리 비용 모델 (수정 없음)
        rewards = {0: 1000.0, 1: 4000.0, 2: 10000.0}
        dist = np.linalg.norm(current_pos - task_pos)
        return rewards[task_id] - (dist * 100.0)

    def phase1_bundle_building(self, tasks, current_pos):
        if len(self.bundle) < 1:
            best_task, best_score = -1, -1e9
            for t_idx, t_pos in tasks.items():
                score = self.calculate_score(t_idx, t_pos, current_pos)
                if score > self.winning_bids[t_idx] + 1e-4:
                    if score > best_score:
                        best_score, best_task = score, t_idx
            if best_task != -1:
                self.bundle = [best_task]
                self.winning_bids[best_task], self.winners[best_task] = best_score, self.id

    def phase2_consensus(self, all_agents):
        # PDF p.94: Winning Bids 기반 충돌 해결 (수정 없음)
        changed = False
        for t in range(self.num_tasks):
            for other in all_agents:
                if other.id == self.id:
                    continue
                if other.winning_bids[t] > self.winning_bids[t] + 1e-4:
                    self.winning_bids[t], self.winners[t] = other.winning_bids[t], other.winners[t]
                    if t in self.bundle:
                        self.bundle = []
                        changed = True
        return changed


def main():
    swarm = Crazyswarm()
    helper = swarm.allcfs
    time_helper = swarm.timeHelper
    drones = {i + 1: cf for i, cf in enumerate(helper.crazyflies)}

    TASKS = {
        0: np.array([0.0, 0.0, 1.0]),
        1: np.array([2.5, 0.0, 1.0]),
        2: np.array([5.0, 0.0, 1.0])
    }

    print("\n[Step 1] 대형 형성 및 안정화")
    helper.takeoff(targetHeight=1.0, duration=2.0)
    time_helper.sleep(2.0)
    for i in range(1, 4):
        drones[i].goTo(TASKS[i-1], yaw=0.0, duration=4.0)

    # ---------------------------------------------------------
    # [수정 구간] 기존 40초 대기를 삭제하고 사용자 입력을 기다림
    print("\n" + "!"*60)
    print(" >>> 드론 3번을 추락시키려면 터미널에 [Enter]를 입력하세요 <<<")
    print("!"*60)
    input()  # 여기서 키보드 입력을 무한 대기합니다.
    # ---------------------------------------------------------

    print("\n[Step 2] 3번 드론 추락 상황 인지 (Crash Simulation)")
    drones[3].land(targetHeight=0.0, duration=1.5)
    time_helper.sleep(5.0)

    print("\n[Step 3] CBBA 협상 개시")
    active_ids = [1, 2]
    agents = {i: CBBAAgent(i, len(TASKS)) for i in active_ids}
    for _ in range(50):
        for i in active_ids:
            agents[i].phase1_bundle_building(TASKS, TASKS[i-1])
        for i in active_ids:
            agents[i].phase2_consensus(list(agents.values()))

    print("\n" + "="*50)
    print(" CBBA 최종 임무 재할당 결과 ")
    print("="*50)
    for i in active_ids:
        t_idx = agents[i].bundle[0] if agents[i].bundle else "None"
        print(f" 드론 {i}번: Task {t_idx} 낙찰 확정")
    print("="*50)

    print("\n[Step 4] 전 기체 동시 이동 (초고속 모드)")
    for i in active_ids:
        if agents[i].bundle:
            t_idx = agents[i].bundle[0]
            if np.linalg.norm(TASKS[i-1] - TASKS[t_idx]) > 0.5:
                print(f" 드론 {i}번: Task {t_idx}로 초고속 전진!")
                drones[i].goTo(TASKS[t_idx], yaw=0.0, duration=2.5)

    print("\n[Step 5] 재구성 완료 및 정찰 호버링 (80초 대기)")
    time_helper.sleep(80.0)
    print(" 미션 완료. 착륙합니다.")
    helper.land(targetHeight=0.0, duration=3.0)


if __name__ == "__main__":
    main()
