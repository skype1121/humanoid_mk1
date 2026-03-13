# humanoid_control AK70 multi-motor verification notes

## 1. 현재 코드상 확인된 것

- 멀티 드라이버 노드: `ak70_driver_node.py`
- 레거시 단일 모터 노드: `ak70_can_node.py`
- 멀티 모터 command topic:
  - `/ak70/command` (`sensor_msgs/msg/JointState`)
  - `/ak70/motor_commands` (`humanoid_control/msg/MotorCommandArray`)
- 멀티 모터 state topic:
  - `/ak70/motor_states` (`sensor_msgs/msg/JointState`)
  - `/ak70/motor_states_detailed` (`humanoid_control/msg/MotorStateArray`)
- 공통 driver command topic:
  - `/ak70/driver_command` (`humanoid_control/msg/DriverCommand`)
- 레거시 단일 모터 topic:
  - `/control_command` (`std_msgs/msg/Float64`)
  - `/motor_state` (`std_msgs/msg/Float64`)

위 항목은 코드상 존재가 확인된 내용이다.
반면 실제 모터 방향, 영점, 토크 반응, 링크 간섭, CAN 배선 상태는 실기에서만 확인 가능하다.

## 2. 실기 전 안전 체크

- 처음 검증에서는 `enable_motors_on_startup=false`, `send_zero_on_startup=false` 유지
- `directions`, `zero_offsets`, `min_positions`, `max_positions` 는 현재 임시값으로 보고 시작
- 실제 모터와 조인트 매핑이 아래와 정확히 일치하는지 육안으로 먼저 확인
  - `1 = hip_pitch`
  - `2 = hip_roll`
  - `3 = hip_yaw`
  - `4 = knee_pitch`
  - `5 = ankle_pitch`
- 첫 position 명령은 `0.02 rad` 수준의 작은 단발 명령만 사용
- 처음에는 UI보다 CLI 단발 명령으로 검증
- 기구가 공중에 떠 있거나, 충돌 없이 수동으로 막을 수 있는 상태에서 시작

## 3. 단계별 검증 절차

### 3-1. 공통 준비

```bash
source /opt/ros/humble/setup.bash
source /tmp/hc_install/local_setup.bash
ros2 launch humanoid_control ak70_driver.launch.py
```

별도 터미널에서 기본 상태 확인:

```bash
ros2 node list
ros2 topic list | grep ak70
ros2 topic info /ak70/motor_states_detailed
ros2 topic echo /ak70/motor_states_detailed --once
```

CAN 프레임 확인:

```bash
candump can0
```

기대 포인트:

- `ros2 node list` 에 `/ak70_driver_node` 존재
- `/ak70/motor_states_detailed` 가 publish 중
- `candump can0` 에 송수신 프레임이 보임
- 로그에 `Unexpected feedback id`, `Feedback id mismatch`, `Failed to parse feedback frame` 가 반복되지 않음

### 3-2. 1개 모터 검증

권장:

- YAML에서 먼저 1개 모터만 남기거나
- 물리적으로 1개만 연결한 상태에서 시작

확인 순서:

1. launch 직후 enable 없이 상태 토픽이 들어오는지 본다.
2. `candump can0` 로 feedback id가 기대한 motor id와 같은지 본다.
3. enable 후 `0.02 rad` 단발 명령을 1회 보낸다.
4. 방향이 맞는지 확인한다.
5. 다시 `0.00 rad` 로 복귀시킨다.

명령 예시:

```bash
ros2 topic pub --once /ak70/driver_command humanoid_control/msg/DriverCommand \
"{command: 'ENABLE_ALL'}"
```

```bash
ros2 topic pub --once /ak70/motor_commands humanoid_control/msg/MotorCommandArray \
"{stamp: {sec: 0, nanosec: 0}, commands: [{motor_id: 1, joint_name: 'hip_pitch', target_position: 0.02, kp: 0.0, kd: 0.0, has_gains: false}]}"
```

```bash
ros2 topic pub --once /ak70/motor_commands humanoid_control/msg/MotorCommandArray \
"{stamp: {sec: 0, nanosec: 0}, commands: [{motor_id: 1, joint_name: 'hip_pitch', target_position: 0.00, kp: 0.0, kd: 0.0, has_gains: false}]}"
```

중점 확인:

- `/ak70/motor_states_detailed` 의 `motor_id`, `joint_name`, `position`
- `candump can0` 상 송신 id와 feedback id
- 로그에 `Command clamp applied` 또는 `Command step limited` 가 뜨면 요청값이 제한되고 있다는 뜻

### 3-3. 2개 모터 검증

권장 조합:

- `hip_pitch`, `hip_roll` 같이 구분이 쉬운 2개부터

확인 순서:

1. 두 모터만 연결하거나 YAML을 2개 기준으로 맞춘다.
2. 두 모터 모두 enable 후 한 번에 하나씩만 `0.02 rad` 명령을 보낸다.
3. 첫 번째 모터 정상 확인 후 두 번째 모터를 확인한다.
4. 마지막에 두 모터 동시 명령을 아주 작게 보낸다.

동시 명령 예시:

```bash
ros2 topic pub --once /ak70/motor_commands humanoid_control/msg/MotorCommandArray \
"{stamp: {sec: 0, nanosec: 0}, commands: [{motor_id: 1, joint_name: 'hip_pitch', target_position: 0.02, kp: 0.0, kd: 0.0, has_gains: false}, {motor_id: 2, joint_name: 'hip_roll', target_position: -0.02, kp: 0.0, kd: 0.0, has_gains: false}]}"
```

추가 확인:

- 한 모터 명령 시 다른 모터 상태가 예상치 않게 튀지 않는지
- feedback 누락 경고가 특정 id에서만 반복되는지
- `candump can0` 상 두 id가 모두 보이는지

### 3-4. 5개 모터 검증

전제:

- 1개, 2개 단계에서 방향과 영점이 각각 확인된 뒤에만 진행

확인 순서:

1. 5개 모두 연결 후 enable 없이 state 수신 확인
2. 5개 모두 enable
3. 각 조인트를 순서대로 `0.02 rad` 단발 명령으로 개별 확인
4. 모든 조인트가 맞으면 5개 동시 `0.02 rad` 이하의 작은 명령 확인
5. 마지막에 `STOP` 또는 `0.00 rad` 복귀

5개 동시 명령 예시:

```bash
ros2 topic pub --once /ak70/motor_commands humanoid_control/msg/MotorCommandArray \
"{stamp: {sec: 0, nanosec: 0}, commands: [{motor_id: 1, joint_name: 'hip_pitch', target_position: 0.02, kp: 0.0, kd: 0.0, has_gains: false}, {motor_id: 2, joint_name: 'hip_roll', target_position: 0.02, kp: 0.0, kd: 0.0, has_gains: false}, {motor_id: 3, joint_name: 'hip_yaw', target_position: 0.02, kp: 0.0, kd: 0.0, has_gains: false}, {motor_id: 4, joint_name: 'knee_pitch', target_position: 0.02, kp: 0.0, kd: 0.0, has_gains: false}, {motor_id: 5, joint_name: 'ankle_pitch', target_position: 0.02, kp: 0.0, kd: 0.0, has_gains: false}]}"
```

정지 예시:

```bash
ros2 topic pub --once /ak70/driver_command humanoid_control/msg/DriverCommand \
"{command: 'STOP'}"
```

```bash
ros2 topic pub --once /ak70/driver_command humanoid_control/msg/DriverCommand \
"{command: 'DISABLE_ALL'}"
```

## 4. CLI 점검 명령 모음

### 노드/토픽 확인

```bash
ros2 node list
ros2 topic list | grep -E 'ak70|motor_state|control_command'
ros2 topic info /ak70/motor_commands
ros2 topic info /ak70/motor_states_detailed
ros2 interface show humanoid_control/msg/DriverCommand
ros2 interface show humanoid_control/msg/MotorCommandArray
ros2 interface show humanoid_control/msg/MotorStateArray
```

### 상태 확인

```bash
ros2 topic echo /ak70/motor_states_detailed
```

```bash
ros2 topic echo /ak70/motor_states --once
```

```bash
ros2 topic hz /ak70/motor_states_detailed
```

### Driver command

```bash
ros2 topic pub --once /ak70/driver_command humanoid_control/msg/DriverCommand \
"{command: 'ENABLE_ALL'}"
```

```bash
ros2 topic pub --once /ak70/driver_command humanoid_control/msg/DriverCommand \
"{command: 'ZERO_POSE', joint_names: ['hip_pitch', 'hip_roll', 'hip_yaw', 'knee_pitch', 'ankle_pitch'], target_positions: [0.0, 0.0, 0.0, 0.0, 0.0]}"
```

```bash
ros2 topic pub --once /ak70/driver_command humanoid_control/msg/DriverCommand \
"{command: 'STOP'}"
```

### Motor command

```bash
ros2 topic pub --once /ak70/motor_commands humanoid_control/msg/MotorCommandArray \
"{stamp: {sec: 0, nanosec: 0}, commands: [{motor_id: 1, joint_name: 'hip_pitch', target_position: 0.02, kp: 0.0, kd: 0.0, has_gains: false}]}"
```

```bash
ros2 topic pub --once /ak70/motor_commands humanoid_control/msg/MotorCommandArray \
"{stamp: {sec: 0, nanosec: 0}, commands: [{motor_id: 1, joint_name: 'hip_pitch', target_position: 0.02, kp: 0.0, kd: 0.0, has_gains: false}, {motor_id: 2, joint_name: 'hip_roll', target_position: -0.02, kp: 0.0, kd: 0.0, has_gains: false}]}"
```

## 5. 런타임 로그 해석

- `No recent feedback received for ...`
  - enable된 모터에서 최근 feedback이 안 들어온 상태
- `Unexpected feedback id on can0 ...`
  - 설정에 없는 CAN ID가 들어온 상태
- `Feedback id mismatch ...`
  - CAN arbitration id와 payload 내부 id가 다름
- `Command clamp applied ...`
  - 요청한 각도가 YAML 제한 범위를 넘어서 잘린 상태
- `Command step limited ...`
  - 목표 변화량이 커서 한 주기당 이동량 제한이 걸린 상태

위 로그는 이해를 돕기 위한 최소 수준으로만 남겨 두었고, 반복 출력은 1초 단위로 제한했다.

## 6. 실기 전까지 단정할 수 없는 것

아래는 아직 코드만으로는 성공을 단정할 수 없다.

- 각 조인트의 실제 회전 방향이 `directions` 와 맞는지
- `zero_offsets` 가 실제 영점과 맞는지
- `min_positions`, `max_positions` 가 기구 간섭 없이 안전한지
- 5개 동시 구동 시 CAN 부하와 feedback 누락이 허용 범위인지
- hip_yaw를 포함한 실제 배선/기구 매핑이 YAML과 정확히 일치하는지

따라서 현재 상태는 "코드상 멀티 모터 검증 경로와 로그 보강이 준비된 상태"이지,
"실제 하드웨어 동작 성공이 확인된 상태"는 아니다.
