# FFW-SH5 양손 제어

MuJoCo에서 **패널 1(오른손)**과 **패널 3(왼손)**의 각 6개 슬라이더로 손의 위치·방향을 조절합니다.
공통 제약 미분 IK/HQP가 팔을 먼저 사용하고, 필요한 경우 반대 손 높이를 유지하며 공유 리프트를 움직입니다.
관절 목표는 힘 제한을 적용한 PD와 중력·코리올리·원심력 보상으로 추종합니다.

**패널 2(오른손)**와 **패널 4(왼손)**에서는 엄지와 나머지 네 손가락을 각각 접고 펼 수 있습니다.
**패널 5**는 전체 조인트 상태와 직접 제어, **패널 6**은 시뮬레이션 시간·IK 오차·루프 주파수를 표시합니다.
0.8m 정육면체와 자유 캔 두 개를 배치했으며 손가락의 물리 접촉으로 잡고 들어 올릴 수 있습니다.
패널 6의 픽업 버튼으로 양손의 접근·파지·들기를 바로 확인할 수 있습니다. 자동 충돌 회피는 후속 범위입니다.

## 실행

프로젝트 루트에서 기존 가상환경을 활성화합니다.

```bash
source .venv/bin/activate
python -m pip install -e .
ffw-control
```

설치된 환경에서는 활성화 후 `ffw-control`만 실행하면 됩니다.
`python -m ffw_control.app`으로도 실행할 수 있습니다.
GUI 창을 열 수 있는 데스크톱 터미널에서 실행하세요.

새 환경을 만드는 경우:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
```

기본 패키지는 기존 `requirements-ffw.txt`, 추가 솔버 패키지는 `requirements-control.txt`에서 읽습니다.
가상환경 폴더 이름은 `.venv`이며 종료는 `deactivate`입니다.
VS Code 인터프리터는 `.venv/bin/python`을 선택합니다.

## 조작

- 왼쪽 위 패널 1은 오른손, 패널 3은 왼손입니다. 각각 X/Y/Z와 Roll/Pitch/Yaw를 조절합니다.
- 패널 2·4의 `Thumb`는 엄지, `Four fingers`는 나머지 네 손가락입니다.
  `0`은 편 자세, `1`은 설정된 접은 자세이며 중간값으로 접는 정도를 조절합니다.
  손가락 입력은 활성 손이나 팔 목표를 바꾸지 않으므로 양손을 독립적으로 조절할 수 있습니다.
- X/Y/Z는 월드 좌표의 절대 위치(m)입니다. 초기 베이스는 고정되어 있습니다.
- 회전은 초기 손 방향에 대한 변화량(도)입니다. 월드 고정축 X/Y/Z의 회전을 순서대로 적용합니다.
- 마지막으로 조작한 손이 활성 손입니다. 반대 손은 그 시점의 실제 높이를 유지합니다.
  반대 손의 X/Y 위치·방향은 낮은 우선순위로 유지하려고 시도하며, 고정 조건은 높이입니다.
- `ARM`은 리프트 고정, `LIFT`는 양팔·리프트 동시 계산입니다.
  팔 도달성 검사는 백그라운드에서 수행하여 슬라이더 입력을 막지 않습니다.
- 패널 5의 `Filter`와 목록 스크롤로 63개 조인트를 확인합니다. 마우스를 올리면 실제 위치·속도·구동기 힘·목표값이 보입니다.
- `Direct joints`를 켜면 패널 1~4를 멈추고 개별 조인트를 제어합니다. 바퀴 구동은 `v=` 속도(rad/s), 나머지는 위치(rad/m)입니다.
  끄면 현재 실제 자세에서 IK를 재개하며, 개별 손가락 자세는 다음 프리셋 입력까지 유지합니다.
- 패널 5의 `Pause`/`Resume`으로 물리 진행을 멈추거나 재개하고, `Reset`으로 초기 상태로 돌아갑니다.
- 패널 6의 `Pick right can` / `Pick left can`은 초기 배치에서 접근·파지·들기·3초 유지를 진행합니다.
  완료 후 패널 2·4의 두 슬라이더를 0으로 되돌리면 캔을 놓습니다. 재실행은 `Reset` 후 진행합니다.
  수동 입력은 시퀀스를 취소하고 우선 적용됩니다. `Cancel sequence`는 마지막 제어 목표를 유지합니다.
- 장면에서 왼쪽 드래그는 시점 회전, 오른쪽 드래그는 이동, 마우스 휠은 확대·축소입니다. Esc로 종료합니다.

패널 6은 오른손 기준 IK 오차, 시뮬레이션/실제 시간, 실제 루프 주파수와 EMA를 표시합니다.
실제 추종 오차, 반대 손 높이 오차, 리프트 위치, 양팔·리프트 구동기 포화 개수도 보여줍니다.
각 캔의 접촉·들림 상태와 작업대 기준 이격도 표시합니다. 물성과 판정 기준은 [파지 안내](docs/grasping.md)에 정리했습니다.
직접 제어 중 IK 오차는 `N/A`입니다. [패널 5·6 안내](docs/joint_monitor.md)에 지표와 모드 전환 규칙을 정리했습니다.
`IK reached / settling`은 계산상 목표에는 도달했지만 물리 추종이 아직 진행 중이라는 뜻입니다.
`Tracking / limited`에서는 목표를 추종 중이거나 현재 제약 때문에 잔차가 남을 수 있습니다.
QP가 실패하면 `STOP` 상태에서 마지막 유효 관절 목표를 유지합니다. 새 목표 또는 Reset으로 다시 시도할 수 있습니다.

## 화면 없는 실행과 검증

```bash
# 초기 자세 유지
python -m ffw_control.app --headless --seconds 3

# 오른손: 초기 위치에서 X +4cm, Z +3cm, Roll +5도
python -m ffw_control.app --headless --seconds 4 --offset 0.04 0 0.03 --rpy 5 0 0

# 리프트가 필요한 목표: 오른손 Z -10cm
python -m ffw_control.app --headless --seconds 6 --offset 0 0 -0.10

# 왼손 조작
python -m ffw_control.app --headless --seconds 4 --side left --offset 0.03 0 0.03

# 양손 손가락: 엄지와 나머지 네 손가락의 접는 정도
python -m ffw_control.app --headless --seconds 3 --right-fingers 1 1 --left-fingers 0.6 0.8

# 조인트 직접 제어
ffw-control --headless --seconds 3 --joint head_joint1=0.2 --joint finger_r_joint6=0.6

# 물리 접촉으로 캔 파지·들기·유지 (기본 25초)
ffw-control --headless --grasp-demo right
ffw-control --headless --grasp-demo left

# 회귀 테스트
python -m unittest discover -s tests -v
```

`--seconds`는 시뮬레이션 시간입니다. Headless 실행은 실시간 대기 없이 계산하며 종료 시 오차와 상태를 JSON으로 출력합니다.
GUI를 정해진 프레임 뒤 종료하고 캡처할 수도 있습니다.

```bash
ffw-control --frames 120 --screenshot outputs/panels.png
```

ROS를 불러온 터미널에서는 `PYTHONPATH`가 가상환경에도 상속됩니다.
프로젝트 패키지만 대상으로 의존성을 점검하려면 다음을 실행합니다.

```bash
env -u PYTHONPATH .venv/bin/python -m pip check
```

## 폴더 구조

```text
mujoco_proj/
├── .venv/                       # 로컬 가상환경 (Git 제외)
├── requirements-ffw.txt          # 기존 MuJoCo/UI 패키지
├── requirements-control.txt      # OSQP/SciPy 버전
├── pyproject.toml                # 패키지 및 실행 명령
├── robotis_ffw/                  # 기존 XML·메쉬 (원본 유지)
├── configs/
│   ├── robot.json                # 로봇 이름·경로 매핑
│   ├── control.json              # 초기 자세, 제한, 오차, 제어 게인
│   └── fingers.json              # 손가락 펴기/접기 자세 및 명령 속도 제한
├── src/ffw_control/
│   ├── app.py                    # GUI/headless 진입점
│   ├── robots/model.py           # 고정 베이스 모델·손 기준점·Jacobian
│   ├── control/
│   │   ├── fingers.py            # 엄지/네 손가락 독립 제어
│   │   ├── joints.py             # 전체 조인트 조회·직접 제어
│   │   ├── hierarchy/qp.py       # 제약 QP 및 null space 계층
│   │   ├── ik/differential.py    # 미분 IK·팔 도달성 검사
│   │   └── dynamics/servo.py     # 힘 제한 PD 및 bias 보상
│   ├── simulation/session.py    # 양손 목표·모드·물리 진행
│   ├── simulation/timing.py     # 실제 루프 주파수·EMA
│   ├── simulation/grasp.py      # 기존 제어 경로를 사용하는 픽업 시퀀스
│   ├── simulation/objects.py    # 캔 접촉·들림 관측
│   ├── ui/window.py             # 슬라이더·MuJoCo 렌더링
│   └── evaluation/              # 향후 평가 확장
├── scripts/check_model.py        # 기존 원본 모델 점검
├── tests/test_control.py         # 지속적인 회귀 검증
├── scenes/grasp.xml              # 정육면체·자유 캔 두 개·접촉 설정
├── experiments/                 # 향후 실험 설정
├── outputs/                     # 생성 로그·영상·캡처 (Git 제외)
└── docs/                        # 프로젝트 Markdown 문서
```

예정된 전체 기능은 [사용자 기능 명세](docs/functional_spec.md),
모듈의 역할은 [구조 설계](docs/architecture.md),
현재 제어 방식과 제한은 [패널 1·3 제어 문서](docs/panel1_control_design.md)를 참고하세요.
손가락의 조작과 설정은 [패널 2·4 제어 문서](docs/finger_control.md)에 정리했습니다.
기존 `robotis_ffw/README.md`는 이전 예제 안내이므로 현재 실행 방법은 이 문서를 기준으로 합니다.
