"""
Crazyswarm2 간단 예제: 이륙 -> 앞으로 30cm 이동 -> 착륙
"""

from crazyflie_py import Crazyswarm


def main():
    # Crazyswarm 초기화
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # 드론이 최소 2대 있는지 확인
    if len(allcfs.crazyflies) < 2:
        print("에러: 연결된 드론이 2대 미만입니다.")
        return

    # 드론 객체 각각 할당 (개별 제어 필요 시 사용)
    cf1 = allcfs.crazyflies[0]
    cf2 = allcfs.crazyflies[1]

    # 1. 모든 드론 동시 이륙
    print("모든 드론 이륙 시작...")
    allcfs.takeoff(targetHeight=0.5, duration=2.5)

    timeHelper.sleep(4.0)

    # 2. 앞으로 30cm 이동 (상대 좌표 사용)
    # relative=True 이므로 현재 위치에서 x축 방향으로 0.3m 이동합니다.
    print("모든 드론 앞으로 30cm 이동...")
    for cf in allcfs.crazyflies:
        cf.goTo([0.3, 0.0, 0.0], yaw=0.0, duration=2.0, relative=True)
    timeHelper.sleep(3.0)

    # (선택 사항) 만약 2번 드론만 따로 움직이고 싶다면 아래처럼 작성합니다.
    # print("2번 드론만 추가로 옆으로 이동...")
    # cf2.goTo(pos=[0.0, 0.3, 0.0], yaw=0.0, duration=2.0, relative=True)
    # timeHelper.sleep(3.0)

    # 3. 모든 드론 동시 착륙
    print("모든 드론 착륙 시작...")
    allcfs.land(targetHeight=0.0, duration=2.5)
    timeHelper.sleep(3.0)

    print("임무 완료!")


if __name__ == '__main__':
    main()
