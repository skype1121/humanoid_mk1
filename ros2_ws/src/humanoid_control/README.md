# humanoid_control AK70 driver notes

## Topics

- Command topic: `/ak70/command` (`sensor_msgs/msg/JointState`)
- State topic: `/ak70/motor_states` (`sensor_msgs/msg/JointState`)
- Legacy single-motor test:
  - Command: `/control_command` (`std_msgs/msg/Float64`)
  - State: `/motor_state` (`std_msgs/msg/Float64`)

## JointState command behavior

- `name[i]` 와 `position[i]` 가 매칭되는 joint만 target이 갱신된다.
- 메시지에 없는 나머지 joint는 마지막 target을 그대로 유지한다.

예시:

```bash
ros2 topic pub /ak70/command sensor_msgs/msg/JointState \
"{name: ['left_hip_roll'], position: [0.1]}"
```

## Startup parameters

- 권장 시작 순서:
  1. `enable_motors_on_startup=false`
  2. `send_zero_on_startup=false`
  3. 상태 토픽과 피드백 방향 확인
  4. 이후 `enable_motors_on_startup=true` 로 enable 시험
  5. 필요할 때만 `send_zero_on_startup=true` 사용

- `send_zero_on_startup=true` 는 자동 동작이므로 상시 기본값으로 두지 않는다.

## Scale-up procedure

### 1 motor

- YAML에 motor 1개만 둔다.
- startup 두 파라미터를 둘 다 `false`로 둔다.
- launch 후 `/ak70/motor_states` 수신 여부 확인
- 그 다음 enable만 켜서 시험
- 마지막으로 필요 시 zero command를 시험

### 2 motors

- 병렬 배열 길이를 2로 늘린다.
- 두 joint의 `motor_id`, `direction`, `zero_offset` 을 각각 확인한다.
- 한 번에 큰 명령을 보내지 말고 joint 하나씩 소각도 명령으로 확인한다.

### 8 motors

- 2모터 검증이 끝난 설정을 기준으로 배열을 8개까지 확장한다.
- bus 부하와 feedback 누락 여부를 먼저 본다.
- 전체 enable/zeroing 자동화는 마지막 단계에서만 검토한다.

---

# 한국어 요약

## 1. 이 문서의 목적

이 문서는 `humanoid_control`의 AK70 멀티 드라이버를 처음 검증할 때,
어떤 토픽을 쓰는지, 명령이 어떻게 들어가는지, 그리고 모터를 1개부터 8개까지 어떻게 안전하게 늘려서 확인할지를 빠르게 이해하기 위한 요약이다.

## 2. 사용되는 토픽

- 멀티 드라이버 명령 토픽: `/ak70/command`
  - 타입: `sensor_msgs/msg/JointState`
- 멀티 드라이버 상태 토픽: `/ak70/motor_states`
  - 타입: `sensor_msgs/msg/JointState`
- 기존 단일 모터 테스트용 명령 토픽: `/control_command`
  - 타입: `std_msgs/msg/Float64`
- 기존 단일 모터 테스트용 상태 토픽: `/motor_state`
  - 타입: `std_msgs/msg/Float64`

짧은 명령 예시:

```bash
ros2 topic pub /ak70/command sensor_msgs/msg/JointState "{name: ['left_hip_roll'], position: [0.1]}"
```

## 3. JointState 명령 방식 핵심

- `name`과 `position`은 같은 순서끼리 짝으로 처리된다.
- 메시지에 들어온 조인트만 목표값이 갱신된다.
- 메시지에 없는 다른 조인트는 이전 목표값을 그대로 유지한다.

즉, 일부 조인트만 보내도 되고, 안 보낸 조인트가 자동으로 0으로 바뀌지는 않는다.

## 4. startup 파라미터 사용 순서

권장 순서는 아래와 같다.

1. 처음에는 `enable_motors_on_startup=false`
2. 처음에는 `send_zero_on_startup=false`
3. 먼저 상태 토픽과 피드백이 정상인지 확인
4. 그다음 `enable_motors_on_startup=true`로 모터 enable 시험
5. 정말 필요할 때만 `send_zero_on_startup=true` 사용

핵심은 처음부터 자동 enable, 자동 zero를 켜지 않는 것이다.

## 5. 1모터 → 2모터 → 8모터 검증 흐름

### 1모터

- YAML에 모터 1개만 넣고 시작
- startup 자동 동작은 모두 끔
- 상태 토픽이 정상인지 먼저 확인
- 이후 작은 명령으로 천천히 시험

### 2모터

- YAML 배열 길이를 2로 늘림
- 각 모터의 ID, 방향, 오프셋을 개별 확인
- 한 번에 크게 움직이지 말고 하나씩 확인

### 8모터

- 2모터 검증이 끝난 뒤 확장
- 피드백 누락이나 버스 부하를 먼저 확인
- 전체 자동 enable/zero는 가장 마지막에 검토

## 6. 지금 당장 사용자가 기억해야 할 핵심 3줄

- 처음 검증은 무조건 1모터, 자동 enable/zero는 둘 다 끄고 시작한다.
- `JointState` 명령은 들어온 조인트만 갱신되고, 나머지는 이전 목표를 유지한다.
- 1개가 안정되면 2개, 그다음 8개로 늘려야 한다.
