# humanoid_mk1

## Overview

`humanoid_mk1` is a humanoid robotics development repository that combines ROS2-based control software, simulation assets, and project documentation in a single workspace.

## Hardware

- Humanoid robot platform
- Motor control interfaces
- Embedded firmware targets
- Sensors and bring-up peripherals

## Software stack

- ROS2 Humble
- Isaac Sim 5.1.0
- C++
- Python

## Repository structure

- `control/`: Control-related resources and utilities
- `docker/`: Container environment setup
- `docs/`: Project documentation and development notes
- `experiments/`: Experimental code and prototypes
- `firmware/`: Embedded and low-level firmware sources
- `ros2_ws/`: ROS2 workspace
- `simulation/`: Simulation assets, worlds, robots, and tasks
- `tools/`: Helper scripts and tooling

## Development workflow

1. Add or update robot code under `ros2_ws/src/`.
2. Keep simulation assets organized under `simulation/`.
3. Record project decisions and hardware details under `docs/`.
4. Build and test ROS2 packages from `ros2_ws/`.
5. Track experimental work separately before merging it into stable packages.

---

## 한국어 요약

### 프로젝트 목적

`humanoid_mk1`은 휴머노이드 로봇 개발을 위한 통합 저장소입니다.
ROS2 제어 코드, 시뮬레이션 자산, 펌웨어, 실험 코드, 문서를 한 작업 공간에서 함께 관리하는 것이 목적입니다.

### 현재 개발 진행상황

- Jetson Orin Nano 환경 구성 완료
- ROS2 Humble + Docker 개발환경 구축 완료
- AK70 CAN 통신 정상 확인
- ROS2 단일 모터 제어 성공
- ROS2 topic 기반 제어 구조 구현
- AK70 multi-motor driver 구조 추가
- 1모터 → 2모터 → 8모터 확장 검증 절차 정리
- `humanoid_control` 패키지 README에 AK70 운영 문서 추가

### 주요 디렉토리 역할

- `control/`: 제어 관련 리소스와 유틸리티
- `docker/`: 개발용 컨테이너 환경 설정
- `docs/`: 프로젝트 기록, 결정 사항, 하드웨어 메모
- `experiments/`: 실험 코드와 프로토타입
- `firmware/`: 임베디드 및 저수준 펌웨어
- `ros2_ws/`: ROS2 패키지 개발과 빌드 작업 공간
- `simulation/`: 시뮬레이션 자산, 월드, 로봇 모델, 태스크
- `tools/`: 보조 스크립트와 개발 도구

### 개발 흐름

1. 로봇 코드는 `ros2_ws/src/` 아래에서 개발합니다.
2. 시뮬레이션 관련 자산은 `simulation/` 아래에서 관리합니다.
3. 프로젝트 기록과 하드웨어 정보는 `docs/`에 정리합니다.
4. ROS2 패키지는 `ros2_ws/`에서 빌드하고 테스트합니다.
5. 실험성 작업은 `experiments/`에서 검증한 뒤 안정 코드로 반영합니다.

### README 역할 구분

- 루트 `README.md`: 저장소 전체 목적, 구조, 현재 진행상황을 설명하는 문서
- `ros2_ws/src/humanoid_control/README.md`: AK70 드라이버 토픽, startup 파라미터, 모터 확장 검증 절차를 설명하는 운영 문서
