# Franka Robot Control with RealSense

Franka Emika 로봇 팔과 Intel RealSense 카메라를 활용한 공구 분류 시스템

---

## 프로젝트 관리

### 관리자 정보

| 역할 | 담당 | 책임 |
|------|------|------|
| **중간관리자** | Claude (AI Assistant) | 프로젝트 전반 관리, 코드 리뷰, 브랜치 관리, TODO 추적 |

### 관리자 역할

1. **프로젝트 진행 관리**
   - 전체 TODO 리스트 관리 및 업데이트
   - 진행 상황 추적 및 보고
   - 일정 조율

2. **브랜치 관리**
   - 브랜치별 작업 현황 추적
   - PR 리뷰 및 머지 관리
   - 충돌 해결 지원

3. **코드 품질 관리**
   - 코드 리뷰 수행
   - Naming Convention 준수 확인
   - 코드 스타일 일관성 유지

4. **문서화 관리**
   - README 최신 상태 유지
   - API 문서화 관리
   - 변경 이력 기록

5. **이슈 관리**
   - 버그 추적
   - 개선 사항 기록
   - 우선순위 조정

### 작업 요청 방법

작업자는 다음과 같이 요청해주세요:

```
1. 새 기능 개발: "feature/xxx 브랜치에서 xxx 기능 개발할게요"
2. 버그 수정: "fix/xxx 브랜치에서 xxx 버그 수정할게요"
3. 코드 리뷰 요청: "xxx 브랜치 리뷰 부탁드립니다"
4. 머지 요청: "xxx 브랜치 main에 머지해주세요"
5. TODO 업데이트: "xxx 항목 완료했습니다"
```

---

## 작업자 필독 가이드

> **중요:** 모든 작업자는 작업 전 이 섹션을 반드시 읽어주세요.

### 1. 작업 시작 전 필수 체크리스트

```bash
# ✅ 반드시 수행할 것
□ git fetch origin && git pull origin main  # 최신 코드 동기화
□ README.md 확인하여 본인 브랜치 TODO 확인
□ Conflict 현황 테이블에서 본인 브랜치 상태 확인
□ 관리자 피드백 확인 (있는 경우)
```

### 2. 브랜치 생성 규칙

| 작업 유형 | 브랜치 명명 규칙 | 예시 |
|----------|-----------------|------|
| 새 기능 | `feature/<기능명>` | `feature/gripper-action` |
| 버그 수정 | `fix/<이슈명>` | `fix/queue-deadlock` |
| 문서 작업 | `docs/<문서명>` | `docs/api-guide` |
| 리팩토링 | `refactor/<대상>` | `refactor/motion-queue` |

### 3. 커밋 규칙

**커밋 메시지 형식:**
```
<type>: <subject>

<body> (선택)
```

**Type:**
- `feat`: 새 기능
- `fix`: 버그 수정
- `docs`: 문서
- `refactor`: 리팩토링
- `test`: 테스트
- `chore`: 기타

**예시:**
```bash
git commit -m "feat: MoveGroup 액션 클라이언트 구현"
git commit -m "fix: 큐 동기화 데드락 해결"
```

### 4. 코드 작성 규칙

#### Naming Convention (필수 준수)

| 요소 | 규칙 | 올바른 예 | 잘못된 예 |
|------|------|----------|----------|
| 파일명 | snake_case | `gripper_node.py` | `gripperNode.py` |
| 클래스 | PascalCase | `GripperNode` | `gripper_node` |
| 함수/변수 | snake_case | `open_gripper()` | `openGripper()` |
| 상수 | UPPER_SNAKE | `MAX_FORCE` | `maxForce` |
| private | _ prefix | `_internal_state` | `internalState` |

#### ROS 2 토픽/서비스 명명

```
토픽: <노드명>/<기능>
서비스: <노드명>/<동작>

예시:
- gripper/is_grasping (토픽)
- gripper/open (서비스)
- robot/target_pose (토픽)
- robot/move_home (서비스)
```

### 5. 작업 중 해야 할 것

1. **정기적인 커밋**
   - 논리적 단위로 자주 커밋
   - 하루 작업 끝나면 반드시 푸시

2. **README 업데이트**
   - 본인 브랜치 TODO 체크리스트 업데이트
   - 발견한 개선 사항 기록
   - 이슈 발생 시 기록

3. **main과 동기화**
   ```bash
   # 주기적으로 (최소 하루 1회)
   git fetch origin
   git rebase origin/main
   ```

### 6. PR(Pull Request) 요청 전 체크리스트

```bash
□ 모든 TODO 항목 완료 확인
□ 코드 테스트 완료
□ Naming Convention 준수 확인
□ main과 conflict 없음 확인
□ README의 "리뷰 요청 사항" 작성
□ 커밋 히스토리 정리 (필요시 squash)
```

### 7. 관리자에게 요청해야 하는 것

| 상황 | 요청 방법 |
|------|----------|
| Conflict 해결 어려움 | "xxx 브랜치 conflict 해결 도움 요청" |
| 코드 리뷰 요청 | "xxx 브랜치 리뷰 부탁드립니다" |
| main 머지 요청 | "xxx 브랜치 main 머지 요청" |
| 새 브랜치 TODO 생성 | "feature/xxx 브랜치 TODO 생성 요청" |
| 이슈/버그 보고 | "xxx 이슈 발견: (상세 내용)" |

### 8. 금지 사항

```
❌ main 브랜치에 직접 push 금지
❌ force push는 본인 브랜치에만 (main 절대 금지)
❌ 다른 사람 브랜치 임의 수정 금지
❌ 테스트 안 된 코드 PR 금지
❌ README TODO 업데이트 없이 작업 완료 선언 금지
```

### 9. 문제 발생 시 대응

| 문제 | 대응 방법 |
|------|----------|
| Conflict 발생 | README Conflict 현황에 기록 → 관리자 요청 |
| 빌드 에러 | 본인 브랜치에서 해결 후 푸시 |
| 테스트 실패 | 수정 후 재테스트, PR 보류 |
| 설계 변경 필요 | 관리자와 상의 후 진행 |

### 최근 업데이트 기록

| 날짜 | 내용 | 담당 |
|------|------|------|
| 2024-12-18 | 프로젝트 초기 구조 설정 | Claude |
| 2024-12-18 | gripper_node, robot_control_node 스켈레톤 생성 | Claude |
| 2024-12-18 | README TODO 리스트, 브랜치 관리 섹션 추가 | Claude |
| 2024-12-18 | feature/gripper-libfranka 코드 리뷰 완료 | Claude (관리자) |

---

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
- [x] franka_gripper 액션 클라이언트 연동 *(feature/gripper-libfranka)*
  - [x] `Move` 액션 구현 (열기/닫기)
  - [x] `Grasp` 액션 구현 (파지)
- [x] 파지력 조절 파라미터화 *(feature/gripper-libfranka)*
- [x] 에러 핸들링 강화 *(feature/gripper-libfranka)*
- [x] 단위 테스트 작성 *(feature/gripper-libfranka)*

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

## 브랜치 관리

### 브랜치 전략

| 브랜치 | 용도 | 규칙 |
|--------|------|------|
| `main` | 안정 버전 | PR 통해서만 머지, 직접 푸시 금지 |
| `develop` | 개발 통합 | 기능 브랜치 머지 대상 |
| `feature/*` | 기능 개발 | `feature/gripper-action` 형식 |
| `fix/*` | 버그 수정 | `fix/queue-sync-issue` 형식 |
| `docs/*` | 문서 작업 | `docs/api-documentation` 형식 |

### 브랜치별 작업 현황

> **관리자 Note:** 각 브랜치 작업자는 작업 시작/완료 시 이 섹션을 업데이트해주세요.

#### 현재 활성 브랜치

| 브랜치명 | 담당자 | 작업 내용 | 상태 | 시작일 | 예상 완료 |
|----------|--------|-----------|------|--------|-----------|
| `main` | - | 안정 버전 유지 | ✅ 안정 | - | - |
| `feature/gripper-libfranka` | jse111 | gripper_node libfranka 연동 | ✅ 완료 (PR 대기) | 2024-12-18 | 2024-12-19 |

#### 브랜치별 TODO

<details>
<summary><b>feature/gripper-libfranka</b> (완료 - PR 대기)</summary>

**목표:** franka_gripper 액션 클라이언트 연동 (libfranka SDK)

**담당자:** jse111

**커밋 기록:**
| 커밋 | 설명 |
|------|------|
| `4ef3bc6` | feat: gripper_node에 franka_gripper 액션 클라이언트 연동 |
| `c182e78` | feat: gripper_node 에러 핸들링 강화 |
| `d4731ed` | docs: README TODO 리스트 업데이트 및 브랜치 작업 기록 |
| `b22e8d1` | test: gripper_node 단위 테스트 추가 |

**TODO:**
- [x] Move 액션 클라이언트 구현 (열기/닫기)
- [x] Grasp 액션 클라이언트 구현 (파지)
- [x] 파지력/속도 파라미터화
- [x] 에러 핸들링 강화 (에러 코드, 복구 서비스)
- [x] 단위 테스트 작성 ✅ (2024-12-18)

**구현 내용:**
- `franka_msgs.action.Move` 액션으로 그리퍼 열기/닫기
- `franka_msgs.action.Grasp` 액션으로 물체 파지
- `GripperError` 열거형으로 상세 에러 코드 관리
- `/gripper/recover` 에러 복구 서비스
- `/gripper/stop` 동작 중지 서비스
- `/gripper/state`, `/gripper/error` 상태 토픽

**개선 사항:**
- 액션 서버 미연결 시 시뮬레이션 모드 지원
- busy 상태 관리로 중복 요청 방지

**리뷰 요청 사항:**
- franka_msgs 액션 메시지 필드 확인 필요
- 타임아웃 값 적절성 검토 (현재 10초)

**🔧 관리자 피드백 (2024-12-18):**

✅ **잘된 점:**
- Naming Convention 완벽 준수 (snake_case, PascalCase, UPPER_SNAKE 모두 OK)
- `GripperError` enum으로 상세 에러 코드 관리 - 좋은 설계
- busy 상태 관리로 중복 요청 방지 - 실제 운용시 중요
- 액션 서버 미연결시 시뮬레이션 모드 fallback - 개발/테스트에 유용
- README TODO 업데이트 잘 해주셨습니다

⚠️ **개선 필요:**
1. **`spin_until_future_complete` 블로킹 이슈** (lines 364, 376, 404, 416, 449, 461)
   - 서비스 콜백 내에서 `rclpy.spin_until_future_complete()` 호출은 블로킹 문제 발생 가능
   - MultiThreadedExecutor 사용시 데드락 위험
   - **해결방안:** async/await 패턴 사용 또는 콜백 기반 처리로 변경 권장
   ```python
   # 현재 (블로킹)
   rclpy.spin_until_future_complete(self, future, timeout_sec=...)

   # 권장 (async)
   async def _execute_open_async(self):
       future = self._move_action_client.send_goal_async(goal)
       goal_handle = await future
       result = await goal_handle.get_result_async()
   ```

2. **타임아웃 값 하드코딩**
   - `ACTION_TIMEOUT = 10.0`이 클래스 상수로 되어있으나, 파라미터로 받으면 더 유연

📋 **리뷰 요청 사항 응답:**
- franka_msgs 액션 메시지 필드: `Move.Goal`에 `width`, `speed` 필드 맞음. `Grasp.Goal`에 `epsilon.inner/outer` 접근 방식 확인 필요 (실제 메시지 정의에 따라 다를 수 있음)
- 타임아웃 10초: 일반적인 그리퍼 동작에는 충분하나, 느린 속도 설정시 부족할 수 있음

**📊 리뷰 결과: APPROVE (조건부)**
- blocking 이슈 해결 후 main 머지 가능
- 또는 현재 상태로 develop에 머지하고 추후 개선 가능

---

**🔧 관리자 피드백 - 테스트 리뷰 (2024-12-18):**

✅ **테스트 코드 잘 작성됨!**
- pytest 구조 적절히 사용
- ActionClient mocking으로 단위 테스트 독립성 확보
- 열거형, 초기화, 에러 핸들링, 서비스, 시뮬레이션 모드 모두 커버
- test_ prefix 네이밍 규칙 준수

📝 **추가 권장 테스트:**
1. `_handle_open`, `_handle_close`, `_handle_grasp` 서비스 핸들러 직접 테스트
2. 실제 액션 클라이언트 성공/실패 시나리오 mocking 테스트
3. busy 상태에서 서비스 요청 거부 테스트

**🎉 ALL TODO COMPLETED - PR 준비 완료!**

**⚠️ Conflict 상태:** 없음

</details>

<details>
<summary><b>feature/moveit-integration</b> (예정)</summary>

**목표:** robot_control_node에서 MoveIt 연동

**TODO:**
- [ ] MoveGroup 액션 클라이언트 구현
- [ ] 목표 포즈 전송 구현
- [ ] 실행 결과 피드백 처리
- [ ] joint target 이동 구현
- [ ] cartesian path 이동 구현

**개선 사항:**
- (작업 중 발견된 개선점 기록)

**리뷰 요청 사항:**
- (PR 시 리뷰어가 확인해야 할 사항)

**🔧 관리자 피드백:**
- (관리자가 코드 리뷰 후 피드백 작성)

**⚠️ Conflict 상태:** 없음

</details>

<details>
<summary><b>feature/camera-node</b> (예정)</summary>

**목표:** RealSense 카메라 노드 개발

**TODO:**
- [ ] camera_node 패키지 생성
- [ ] RealSense SDK 연동
- [ ] 포인트 클라우드 퍼블리시
- [ ] RGB/Depth 이미지 퍼블리시
- [ ] 카메라 파라미터 설정

**개선 사항:**
- (작업 중 발견된 개선점 기록)

**리뷰 요청 사항:**
- (PR 시 리뷰어가 확인해야 할 사항)

**🔧 관리자 피드백:**
- (관리자가 코드 리뷰 후 피드백 작성)

**⚠️ Conflict 상태:** 없음

</details>

<details>
<summary><b>feature/perception</b> (예정)</summary>

**목표:** 공구 인식 및 분류 노드 개발

**TODO:**
- [ ] perception_node 패키지 생성
- [ ] 객체 탐지 알고리즘 구현
- [ ] 포즈 추정 구현
- [ ] 공구 분류 알고리즘 구현

**개선 사항:**
- (작업 중 발견된 개선점 기록)

**리뷰 요청 사항:**
- (PR 시 리뷰어가 확인해야 할 사항)

**🔧 관리자 피드백:**
- (관리자가 코드 리뷰 후 피드백 작성)

**⚠️ Conflict 상태:** 없음

</details>

### 브랜치 Conflict 해결 가이드

**Conflict 발생 시 처리 절차:**

```bash
# 1. main 브랜치 최신화
git checkout main
git pull origin main

# 2. 작업 브랜치로 이동
git checkout feature/your-branch

# 3. main 변경사항 rebase (권장) 또는 merge
git rebase main
# 또는
git merge main

# 4. Conflict 발생 파일 확인
git status

# 5. Conflict 해결 후
git add <resolved-files>
git rebase --continue  # rebase인 경우
# 또는
git commit  # merge인 경우

# 6. 강제 푸시 (rebase인 경우)
git push -f origin feature/your-branch
```

**Conflict 해결 요청:**
- 해결이 어려운 경우 관리자에게 요청
- README의 해당 브랜치 섹션에 Conflict 상태 업데이트

### Conflict 현황

| 브랜치 | main과 충돌 | 충돌 파일 | 상태 | 해결 담당 |
|--------|-------------|-----------|------|-----------|
| *(현재 충돌 없음)* | | | | |

### 완료된 브랜치 기록

| 브랜치명 | 담당자 | 작업 내용 | 머지일 | PR 번호 |
|----------|--------|-----------|--------|---------|
| *(완료된 브랜치 기록)* | | | | |

### 브랜치 작업 가이드

**1. 새 브랜치 생성 시:**
```bash
git checkout main
git pull origin main
git checkout -b feature/your-feature-name
```

**2. 작업 시작 시:**
- 위 "현재 활성 브랜치" 테이블에 브랜치 추가
- 해당 브랜치의 TODO 섹션 생성 (위 템플릿 참고)

**3. 작업 중:**
- TODO 체크리스트 업데이트
- 발견된 개선 사항 기록
- 정기적으로 커밋 & 푸시

**4. 작업 완료 시:**
- 모든 TODO 체크 확인
- "리뷰 요청 사항" 작성
- PR 생성 → develop 또는 main으로 머지 요청

**5. 머지 완료 후:**
- "현재 활성 브랜치"에서 제거
- "완료된 브랜치 기록"에 추가

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
