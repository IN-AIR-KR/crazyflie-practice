# Curve

- `curve_with_yaw_change.py`: 머리 방향이 마뀌면서 곡선의 움직임
- `no_yaw_just_s.py`: 머리는 정면으로 고정해두고 움직임만 곡선

## 설정

1. `~/ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml` 경로에 `crazyflies.yaml` 파일을 넣고 colcon 빌드를 다시 진행합니다.
   ```bash
   colcon build --symlink-install
   ```
2. `~/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples` 경로에 예제 파일(.py)을 넣습니다.

## 실행

- [터미널 1] 시뮬레이션 서버 가동

  ```bash
  ros2 launch crazyflie launch.py backend:=sim
  ```

- [터미널 2] 알고리즘 코드 실행

  ```bash
  # curve_with_yaw_change.py
  python3 ~/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/curve_with_yaw_change.py

  # no_yaw_just_s.py
  python3 ~/ros2_ws/src/crazyswarm2/crazyflie_examples/crazyflie_examples/no_yaw_just_s.py
  ```

- [터미널3] rviz2 실행

  ```bash
  rviz2
  ```

  - 이때 rviz2 화면에서
    - Global Options -> Fixed Frame → world 선택
    - Add 버튼 클릭 → TF 선택
