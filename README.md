# Franka Robot Control with RealSense

Franka Emika 로봇 팔과 Intel RealSense 카메라를 활용한 공구 분류 시스템

## 프로젝트 개요

이 프로젝트는 Franka Emika Panda 로봇 팔과 Intel RealSense 깊이 카메라를 사용하여 공구를 자동으로 인식하고 분류하는 시스템입니다.

## 시스템 구성

- **로봇**: Franka Emika Panda 7-DOF 로봇 팔
- **카메라**: Intel RealSense 깊이 카메라
- **그리퍼**: Franka Hand 그리퍼

## 주요 기능

- RealSense 카메라를 통한 공구 인식
- 공구 위치 및 자세 추정
- 로봇 팔을 이용한 공구 픽업
- 공구 종류별 분류 및 배치

---

## TODO List

> **Note:** 작업 시 이 TODO 리스트를 확인하고, 완료된 항목은 체크해주세요.
> 새로운 작업이 추가되면 이 리스트에 반영해주세요.

### 1. 노드 개발

#### gripper_node
- [x] 스켈레톤 코드 작성
- [ ] franka_gripper 액션 클라이언트 연동
  - [ ] `Move` 액션 구현 (열기/닫기)
  - [ ] `Grasp` 액션 구현 (파지)
- [ ] 파지력 조절 파라미터화
- [ ] 에러 핸들링 강화
- [ ] 단위 테스트 작성

#### robot_control_node
- [x] 스켈레톤 코드 작성
- [x] 모션 큐 관리 구현
- [x] 재시도 로직 구현
- [ ] MoveIt 액션 클라이언트 연동
  - [ ] `MoveGroup` 액션 연결
  - [ ] 목표 포즈 전송 구현
  - [ ] 실행 결과 피드백 처리
- [ ] joint target 이동 구현
- [ ] cartesian path 이동 구현
- [ ] 충돌 감지 시 처리 로직
- [ ] 단위 테스트 작성

#### moveit_node (예정)
- [ ] 패키지 생성
- [ ] MoveIt 설정 파일 작성
- [ ] 경로 계획 인터페이스 구현
- [ ] 충돌 회피 설정
- [ ] SRDF/URDF 설정

#### rviz_node (예정)
- [ ] 패키지 생성
- [ ] RViz 설정 파일 작성
- [ ] 시각화 마커 퍼블리셔 구현
- [ ] 인터랙티브 마커 구현

#### camera_node (예정)
- [ ] 패키지 생성
- [ ] RealSense 카메라 연동
- [ ] 포인트 클라우드 퍼블리시
- [ ] RGB 이미지 퍼블리시
- [ ] depth 이미지 퍼블리시

#### perception_node (예정)
- [ ] 패키지 생성
- [ ] 공구 인식 알고리즘 구현
- [ ] 포즈 추정 구현
- [ ] 분류 알고리즘 구현

### 2. Launch 파일
- [ ] `main.launch.py` - 전체 시스템
- [ ] `robot.launch.py` - 로봇 제어 관련
- [ ] `camera.launch.py` - 카메라 관련
- [ ] `rviz.launch.py` - 시각화

### 3. 설정 파일
- [ ] `robot_params.yaml` - 로봇 파라미터
- [ ] `gripper_params.yaml` - 그리퍼 파라미터
- [ ] `camera_params.yaml` - 카메라 파라미터
- [ ] `rviz_config.rviz` - RViz 설정

### 4. 메시지/서비스/액션 정의
- [ ] 커스텀 메시지 정의 (필요시)
- [ ] 커스텀 서비스 정의 (필요시)
- [ ] 커스텀 액션 정의 (필요시)

### 5. 테스트 및 문서화
- [ ] 단위 테스트 작성
- [ ] 통합 테스트 작성
- [ ] API 문서화
- [ ] 사용자 가이드 작성

### 6. 기타
- [ ] CI/CD 파이프라인 설정
- [ ] Docker 환경 구성

---

### 진행 상황 요약

| 항목 | 상태 | 담당 | 비고 |
|------|------|------|------|
| gripper_node 스켈레톤 | ✅ 완료 | - | 기본 구조 완성 |
| robot_control_node 스켈레톤 | ✅ 완료 | - | 큐 관리 포함 |
| MoveIt 연동 | 🔲 대기 | - | robot_control_node에서 구현 필요 |
| moveit_node | 🔲 대기 | - | 패키지 생성 필요 |
| rviz_node | 🔲 대기 | - | 패키지 생성 필요 |
| camera_node | 🔲 대기 | - | RealSense 연동 필요 |
| perception_node | 🔲 대기 | - | 공구 인식 구현 필요 |
| Launch 파일 | 🔲 대기 | - | - |
| 테스트 | 🔲 대기 | - | - |

**범례:** ✅ 완료 | 🔄 진행중 | 🔲 대기

---

## 프로젝트 구조

이 프로젝트는 **멀티 패키지 구조**입니다. `node/` 폴더 안에 각 노드가 **독립적인 ROS 2 패키지**로 존재하며, `colcon build`로 한 번에 빌드됩니다.

```
franka_robot_control_with_realsense/
│
├── node/                                    # ROS 2 패키지들 (colcon workspace)
│   │
│   ├── gripper_node/                        # 패키지: 그리퍼 제어
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   │   └── gripper_node
│   │   └── gripper_node/
│   │       ├── __init__.py
│   │       └── gripper_node.py
│   │
│   ├── robot_control_node/                  # 패키지: MoveIt 인터커넥션 레이어
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/
│   │   │   └── robot_control_node
│   │   └── robot_control_node/
│   │       ├── __init__.py
│   │       └── robot_control_node.py
│   │
│   ├── moveit_node/                         # 패키지: MoveIt 플래닝 (예정)
│   │   └── ...
│   │
│   └── rviz_node/                           # 패키지: RViz 시각화 (예정)
│       └── ...
│
├── launch/                                  # Launch 파일
├── config/                                  # 설정 파일
├── msg/                                     # 커스텀 메시지
├── srv/                                     # 커스텀 서비스
├── action/                                  # 커스텀 액션
├── src/                                     # 공용 라이브러리
├── README.md
└── LICENSE
```

### 폴더 설명

| 폴더 | 설명 |
|------|------|
| `node/` | **ROS 2 패키지 모음**. 각 하위 폴더가 독립적인 패키지 |
| `node/<pkg>/` | 개별 ROS 2 Python 패키지 (package.xml, setup.py 포함) |
| `launch/` | 여러 노드를 한번에 실행하는 launch 파일 |
| `config/` | YAML 설정 파일, RViz 설정 등 |
| `msg/` | 커스텀 ROS 메시지 정의 (.msg) |
| `srv/` | 커스텀 ROS 서비스 정의 (.srv) |
| `action/` | 커스텀 ROS 액션 정의 (.action) |
| `src/` | 공용 라이브러리 및 유틸리티 |

### 패키지 구조 (ROS 2 Python 패키지)

각 노드 패키지는 아래 구조를 따릅니다:

```
<package_name>/
├── package.xml              # ROS 패키지 매니페스트
├── setup.py                 # Python 패키지 설정
├── setup.cfg                # 설치 스크립트 경로
├── resource/
│   └── <package_name>       # ament 리소스 인덱스 (빈 파일)
└── <package_name>/
    ├── __init__.py
    └── <package_name>.py    # 노드 메인 파일
```

---

## Naming Convention

### 파일명

| 유형 | 규칙 | 예시 |
|------|------|------|
| 패키지 폴더 | `snake_case` | `gripper_node`, `robot_control_node` |
| 노드 파일 | `<package_name>.py` | `gripper_node.py` |
| 모듈 파일 | `snake_case.py` | `motion_queue.py` |
| 런치 파일 | `snake_case.launch.py` | `main.launch.py` |
| 설정 파일 | `snake_case.yaml` | `robot_params.yaml` |
| 메시지 | `PascalCase.msg` | `ToolPose.msg` |
| 서비스 | `PascalCase.srv` | `GripperControl.srv` |

### 코드 스타일

| 요소 | 규칙 | 예시 |
|------|------|------|
| 클래스명 | `PascalCase` | `GripperNode`, `MotionQueue` |
| 함수명 | `snake_case` | `open_gripper()`, `execute_motion()` |
| 변수명 | `snake_case` | `current_state`, `target_pose` |
| 상수 | `UPPER_SNAKE_CASE` | `MAX_VELOCITY`, `DEFAULT_FORCE` |
| private 멤버 | `_` prefix | `_current_state`, `_execute_motion()` |
| ROS 토픽 | `<namespace>/<topic>` | `gripper/state`, `robot/status` |
| ROS 서비스 | `<namespace>/<service>` | `gripper/open`, `robot/stop` |

### 노드/패키지 네이밍

| 패키지 | 노드명 | 설명 |
|--------|--------|------|
| `gripper_node` | `gripper_node` | 그리퍼 제어 |
| `robot_control_node` | `robot_control_node` | MoveIt 인터커넥션 레이어 (큐 관리) |
| `moveit_node` | `moveit_node` | 모션 플래닝 |
| `rviz_node` | `rviz_node` | 시각화 |

---

## ROS 노드

### gripper_node
그리퍼 제어를 담당하는 노드

**Services:**
- `gripper/open` - 그리퍼 열기
- `gripper/close` - 그리퍼 닫기
- `gripper/grasp` - 물체 파지

**Publishers:**
- `gripper/is_grasping` (Bool) - 파지 상태
- `gripper/width` (Float64) - 현재 너비

### robot_control_node
MoveIt과의 **인터커넥션 레이어**. 모션 명령을 큐에 저장하고 순차적으로 실행합니다.

**핵심 기능:**
- 모션 명령 큐 관리 (FIFO)
- 실행 상태 모니터링 (pending → executing → success/failed)
- 실패 시 자동 재시도 (최대 3회)
- 순차적 명령 실행 보장

**Services:**
- `robot/move_home` - 홈 위치 이동
- `robot/stop` - 현재 동작 중지
- `robot/clear_queue` - 큐 비우기

**Publishers:**
- `robot/status` (String) - 현재 상태 (idle/executing/queued)
- `robot/current_command` (String) - 실행 중인 명령

**Subscribers:**
- `robot/target_pose` (PoseStamped) - 목표 포즈 수신

---

## 요구 사항

- ROS 2 (Humble 이상 권장)
- Python 3.8+
- libfranka
- franka_ros2
- MoveIt 2
- librealsense2
- realsense-ros

## 설치 및 빌드

```bash
# 워크스페이스에 클론
cd ~/ros2_ws/src
git clone <repository_url>

# node 폴더를 symlink로 연결 (또는 직접 복사)
ln -s franka_robot_control_with_realsense/node/* .

# 의존성 설치
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 빌드 (node 폴더 내 모든 패키지)
colcon build
```

## 실행 방법

### 개별 노드 실행

```bash
source ~/ros2_ws/install/setup.bash

# 그리퍼 노드
ros2 run gripper_node gripper_node

# 로봇 컨트롤 노드
ros2 run robot_control_node robot_control_node
```

### 서비스 호출

```bash
# 그리퍼 열기
ros2 service call /gripper/open std_srvs/srv/Trigger

# 그리퍼 닫기
ros2 service call /gripper/close std_srvs/srv/Trigger

# 홈 위치 이동
ros2 service call /robot/move_home std_srvs/srv/Trigger

# 로봇 정지
ros2 service call /robot/stop std_srvs/srv/Trigger
```

---

## Git 작업 규칙

### 작업 전 필수 사항

**작업 시작 전 반드시 레포지토리 동기화를 수행하세요!**

```bash
# 1. 원격 저장소 최신 정보 가져오기
git fetch origin

# 2. 현재 브랜치 최신화
git pull origin main

# 3. 브랜치 상태 확인
git status
```

### 커밋 메세지 규칙

```
<type>: <subject>

<body>
```

**Type 종류:**

| Type | 설명 |
|------|------|
| `feat` | 새로운 기능 추가 |
| `fix` | 버그 수정 |
| `docs` | 문서 수정 |
| `style` | 코드 포맷팅 (코드 변경 없음) |
| `refactor` | 코드 리팩토링 |
| `test` | 테스트 코드 추가/수정 |
| `chore` | 빌드, 패키지 매니저 설정 등 |

**예시:**
```bash
git commit -m "feat: gripper_node에 파지력 조절 기능 추가"
git commit -m "fix: robot_control_node 큐 동기화 문제 해결"
git commit -m "docs: README 빌드 방법 업데이트"
```

**규칙:**
1. subject는 50자 이내로 작성
2. 한글 또는 영어로 일관되게 작성
3. 마침표 사용하지 않음
4. 명령문 형태로 작성 (예: "추가", "수정", "삭제")

---

## 라이선스

이 프로젝트는 MIT 라이선스를 따릅니다.
