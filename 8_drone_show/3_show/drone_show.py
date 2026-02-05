from crazyflie_py import Crazyswarm


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # 1. 동시 이륙 (0.7m 높이)
    print("Taking off...")
    allcfs.takeoff(targetHeight=0.7, duration=2.0)
    timeHelper.sleep(3.0)

    # 2. 대형 형성: 정삼각형 (정면에서 보이게 y, z축 활용)
    # cf1은 상단, cf2/cf3는 하단 좌우
    print("Forming Triangle...")
    allcfs.crazyflies[0].goTo([0.0, 0.5, 0.5], yaw=0, duration=2.0)
    allcfs.crazyflies[1].goTo([0.0, 0.0, 1.2], yaw=0, duration=2.0)
    allcfs.crazyflies[2].goTo([0.0, -0.5, 0.5], yaw=0, duration=2.0)
    timeHelper.sleep(4.0)

    print("Bigger Triangle...")
    allcfs.crazyflies[0].goTo([0.0, 1.0, 0.3], yaw=0, duration=2.0)
    allcfs.crazyflies[1].goTo([0.0, 0.0, 1.8], yaw=0, duration=2.0)
    allcfs.crazyflies[2].goTo([0.0, -1.0, 0.3], yaw=0, duration=2.0)
    timeHelper.sleep(4.0)

    print("Smaller Triangle...")
    allcfs.crazyflies[0].goTo([0.0, 0.5, 0.5], yaw=0, duration=2.0)
    allcfs.crazyflies[1].goTo([0.0, 0.0, 1.4], yaw=0, duration=2.0)
    allcfs.crazyflies[2].goTo([0.0, -0.5, 0.5], yaw=0, duration=2.0)
    timeHelper.sleep(4.0)

    # # 3. Wave 동작 (상하 교차 움직임)
    # print("Wave movement...")
    # for _ in range(2):
    #     # 1번 내려가고 2,3번 올라가기
    #     allcfs.crazyflies[0].goTo([0.0, 0.0, 0.7], yaw=0, duration=1.5)
    #     allcfs.crazyflies[1].goTo([-0.6, 0.0, 1.5], yaw=0, duration=1.5)
    #     allcfs.crazyflies[2].goTo([0.6, 0.0, 1.5], yaw=0, duration=1.5)
    #     timeHelper.sleep(2.0)

    #     # 반대로 교차
    #     allcfs.crazyflies[0].goTo([0.0, 0.0, 1.5], yaw=0, duration=1.5)
    #     allcfs.crazyflies[1].goTo([-0.6, 0.0, 0.7], yaw=0, duration=1.5)
    #     allcfs.crazyflies[2].goTo([0.6, 0.0, 0.7], yaw=0, duration=1.5)
    #     timeHelper.sleep(2.0)

    # 4. 복귀 및 착륙
    print("Landing...")
    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)


if __name__ == "__main__":
    main()
