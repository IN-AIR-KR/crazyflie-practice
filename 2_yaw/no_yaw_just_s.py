import numpy as np
from crazyflie_py import Crazyswarm
import time

# [수학 엔진] 기존 solve_7th_poly, get_state 함수는 그대로 유지


def solve_7th_poly(p0, v0, a0, p1, v1, a1, T):
    A = np.array([
        [T**4,    T**5,    T**6,    T**7],
        [4*T**3,  5*T**4,  6*T**5,  7*T**6],
        [12*T**2, 20*T**3, 30*T**4, 42*T**5],
        [24*T,    60*T**2, 120*T**3, 210*T**4]
    ])
    B = np.array([
        p1 - (p0 + v0*T + (a0/2.0)*T**2),
        v1 - (v0 + a0*T),
        a1 - a0,
        0.0
    ])
    X = np.linalg.solve(A, B)
    return [float(p0), float(v0), float(a0/2.0), 0.0, float(X[0]), float(X[1]), float(X[2]), float(X[3])]


def get_state(c, t):
    pos = c[0] + c[1]*t + c[2]*t**2 + c[3]*t**3 + c[4]*t**4 + c[5]*t**5 + c[6]*t**6 + c[7]*t**7
    vel = c[1] + 2*c[2]*t + 3*c[3]*t**2 + 4*c[4]*t**3 + 5*c[5]*t**4 + 6*c[6]*t**5 + 7*c[7]*t**6
    acc = 2*c[2] + 6*c[3]*t + 12*c[4]*t**2 + 20*c[5]*t**3 + 30*c[6]*t**4 + 42*c[7]*t**5
    return float(pos), float(vel), float(acc)


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    # --- [사용자 입력 섹션] ---
    # 원하는 좌표 2개를 여기에 입력하세요.
    point1 = [0.2, 0.4]  # 첫 번째 경유지 (x, y)
    point2 = [0.5, -0.2]  # 두 번째 경유지 (x, y)
    final_point = [0.7, 0.0]  # 최종 목적지

    T_SEG = 2.0  # 각 구간당 이동 시간 (속도 조절)
    TARGET_Z = 0.6
    # -----------------------

    WPs = [
        {'p': [0.0, 0.0], 'v': [0.0, 0.0]},     # 시작
        {'p': point1,     'v': [0.1, 0.1]},     # 경유지 1 (통과 속도 부여)
        {'p': point2,     'v': [0.1, -0.1]},    # 경유지 2
        {'p': final_point, 'v': [0.0, 0.0]}      # 끝
    ]

    cf.takeoff(targetHeight=float(TARGET_Z), duration=2.0)
    timeHelper.sleep(4.0)

    # 궤적 플래닝 (Planning)
    segments = []
    for i in range(3):
        c_x = solve_7th_poly(WPs[i]['p'][0], WPs[i]['v'][0], 0.0, WPs[i+1]['p'][0], WPs[i+1]['v'][0], 0.0, T_SEG)
        c_y = solve_7th_poly(WPs[i]['p'][1], WPs[i]['v'][1], 0.0, WPs[i+1]['p'][1], WPs[i+1]['v'][1], 0.0, T_SEG)
        segments.append({'x': c_x, 'y': c_y})

    print(f"궤적 생성 완료: {point1} -> {point2} 경유 비행 시작")

    start_time = timeHelper.time()
    while True:
        t = timeHelper.time() - start_time
        if t > T_SEG * 3:
            break

        idx = min(int(t / T_SEG), 2)
        t_rel = t - (idx * T_SEG)

        x, vx, ax = get_state(segments[idx]['x'], t_rel)
        y, vy, ay = get_state(segments[idx]['y'], t_rel)

        cf.cmdFullState(
            pos=[x, y, float(TARGET_Z)],
            vel=[vx, vy, 0.0],
            acc=[ax, ay, 0.0],  # 미분 평탄성에 따른 가속도 주입
            yaw=0.0, omega=[0.0, 0.0, 0.0]
        )
        timeHelper.sleep(0.01)

    # 안전 착륙
    land_start = timeHelper.time()
    while (timeHelper.time() - land_start) < 2.0:
        tz = TARGET_Z - (TARGET_Z - 0.03) * ((timeHelper.time()-land_start)/2.0)
        cf.cmdFullState(pos=[final_point[0], final_point[1], float(tz)], vel=[
                        0.0, 0.0, -0.1], acc=[0.0, 0.0, 0.0], yaw=0.0, omega=[0.0, 0.0, 0.0])
        timeHelper.sleep(0.02)

    print("미션 종료")


if __name__ == "__main__":
    main()
