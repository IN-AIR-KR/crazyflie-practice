"""
Crazyswarm2 간단 예제: 이륙 -> 앞으로 30cm 이동 -> 착륙
"""

from crazyflie_py import Crazyswarm


def main():
    # Crazyswarm 초기화
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # 첫 번째 드론 선택
    cf = allcfs.crazyflies[0]

    print("이륙 시작...")
    cf.takeoff(targetHeight=0.5, duration=3)
    timeHelper.sleep(5)

    print("앞으로 30cm 이동...")
    cf.goTo([0.3, 0.0, 0.0], yaw=0.0, duration=2.0, relative=True)
    timeHelper.sleep(3)

    print("착륙 시작...")
    cf.land(targetHeight=0.0, duration=3)
    timeHelper.sleep(5)

    print("완료!")


if __name__ == '__main__':
    main()
