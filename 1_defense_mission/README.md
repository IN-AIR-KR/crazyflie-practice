# Defense Mission

3대의 드론이 대형을 형성한 상태에서 하나의 드론이 추락하고, 다른 드론이 추락한 드론의 자리를 메꾸는 협동 예제입니다.

## 설정

1. `~/ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml` 경로에 `crazyflies.yaml` 파일을 넣고 colcon 빌드를 다시 진행합니다.
   ```bash
   colcon build --symlink-install
   ```
2. `~/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/defense_mission.py` 경로에 `defense_mission.py` 파일을 넣습니다.

## 실행

- [터미널 1] 시뮬레이션 서버 가동

  ```bash
  ros2 launch crazyflie launch.py backend:=sim
  ```

- [터미널 2] 알고리즘 코드 실행
  ```bash
  python3 ~/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/defense_mission.py
  ```
- [터미널3] rviz2 실행

  ```bash
  rviz2
  ```

  - 이때 rviz2 화면에서
    - Global Options -> Fixed Frame → world 선택
    - Add 버튼 클릭 → TF 선택
