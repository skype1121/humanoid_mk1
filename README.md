# humanoid_mk1

## 개요 (Overview)

`humanoid_mk1`은 ROS2 기반 제어 소프트웨어, 시뮬레이션 자산, 그리고 프로젝트 문서를 하나의 워크스페이스에서 관리하는 휴머노이드 로봇 개발 저장소입니다.

## 하드웨어 (Hardware)

- 휴머노이드 로봇 플랫폼
- 모터 제어 인터페이스
- 임베디드 펌웨어 대상 장치
- 센서 및 초기 구동(Bring-up) 주변 장치

## 소프트웨어 스택 (Software Stack)

- ROS2 Humble
- Isaac Sim 5.1.0
- C++
- Python

## 저장소 구조 (Repository Structure)

- `control/`: 제어 관련 리소스와 유틸리티
- `docker/`: 개발 환경 컨테이너 설정
- `docs/`: 프로젝트 문서 및 개발 기록
- `experiments/`: 실험용 코드 및 프로토타입
- `firmware/`: 임베디드 및 저수준 펌웨어 소스
- `ros2_ws/`: ROS2 워크스페이스
- `simulation/`: 시뮬레이션 자산, 환경(world), 로봇 모델, 작업(task)
- `tools/`: 보조 스크립트 및 도구

## 개발 워크플로우 (Development Workflow)

1. `ros2_ws/src/` 아래에 로봇 관련 코드를 추가하거나 수정합니다.
2. 시뮬레이션 자산은 `simulation/` 디렉터리 안에서 체계적으로 관리합니다.
3. 프로젝트 결정 사항과 하드웨어 관련 내용은 `docs/`에 기록합니다.
4. `ros2_ws/`에서 ROS2 패키지를 빌드하고 테스트합니다.
5. 실험적인 작업은 `experiments/`에서 먼저 진행한 뒤, 안정화되면 메인 패키지에 통합합니다.

# humanoid_mk1 ## Overview humanoid_mk1 is a humanoid robotics development repository that combines ROS2-based control software, simulation assets, and project documentation in a single workspace. ## Hardware - Humanoid robot platform - Motor control interfaces - Embedded firmware targets - Sensors and bring-up peripherals ## Software stack - ROS2 Humble - Isaac Sim 5.1.0 - C++ - Python ## Repository structure - control/: Control-related resources and utilities - docker/: Container environment setup - docs/: Project documentation and development notes - experiments/: Experimental code and prototypes - firmware/: Embedded and low-level firmware sources - ros2_ws/: ROS2 workspace - simulation/: Simulation assets, worlds, robots, and tasks - tools/: Helper scripts and tooling ## Development workflow 1. Add or update robot code under ros2_ws/src/. 2. Keep simulation assets organized under simulation/. 3. Record project decisions and hardware details under docs/. 4. Build and test ROS2 packages from ros2_ws/. 5. Track experimental work separately before merging it into stable packages.