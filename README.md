# FFW-SH5 양손 제어

MuJoCo에서 ROBOTIS FFW-SH5 로봇의 **양손과 손가락을 슬라이더로 움직이는 프로젝트**입니다.
작업대 위의 캔을 손가락의 물리 접촉으로 잡고 들어 올리는 데모도 제공합니다.

- 오른손과 왼손의 위치·방향, 엄지와 나머지 네 손가락을 각각 조절합니다.
- 양팔과 공유 리프트를 함께 사용해 활성 손의 위치·방향과 반대 손의 XYZ 위치를 추종합니다.
- 전체 조인트를 관찰하거나 직접 제어하고, 화면 없이 시뮬레이션을 실행할 수 있습니다.

로봇의 베이스는 고정되어 있습니다. 장면에는 한 변이 0.8m인 작업대와 자유롭게 움직이는 캔 두 개가 있으며, **자동 충돌 회피는 아직 지원하지 않습니다.**

[빠른 시작](#빠른-시작) · [화면과 조작](#화면과-조작) · [화면 없이 실행하기](#화면-없이-실행하기) · [설정과 코드](#설정과-코드) · [문제 해결](#문제-해결)

## 빠른 시작

Python 3.10 이상이 필요합니다. GUI는 창을 열 수 있는 데스크톱 터미널에서 실행하세요.
아래 명령은 **프로젝트 루트에서** 실행합니다.

### 처음 설치할 때

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
ffw-control
```

이미 `.venv`가 있다면 첫 줄은 건너뛰세요. `pip install -e .`은 [requirements.txt](requirements.txt)에 고정된 의존성을 함께 설치합니다.

### 다음에 실행할 때

```bash
source .venv/bin/activate
ffw-control
```

`python -m ffw_control.app`으로도 실행할 수 있습니다.
가상환경을 나가려면 `deactivate`를 입력합니다. VS Code에서는 인터프리터로 `.venv/bin/python`을 선택하세요.

### 처음 해볼 조작

1. **패널 1**의 X/Y/Z 슬라이더를 조금씩 움직여 오른손 위치를 바꿉니다.
2. **패널 2**의 `Thumb`와 `Four fingers`로 손가락을 접고 펴 봅니다.
3. **패널 5**의 `Reset`으로 초기 상태로 돌아갑니다.
4. **패널 6**에서 오른손 IK 오차와 시간·루프 주파수를 확인합니다.

시작과 `Reset` 직후에는 리프트가 최상단(조인트 위치 0.0m)에 있고, 양팔의 팔꿈치가 90° 굽혀져 있습니다.
손바닥은 서로 마주보고, 네 손가락은 로봇 앞쪽을 향하며, 손목은 꺾이지 않은 자세입니다.
리프트의 이동 범위는 -0.5~0.0m이며, 음수 방향으로 움직이면 상체가 내려갑니다.

## 화면과 조작

### 패널 찾기

| 패널 | 대상 | 할 수 있는 일 |
| --- | --- | --- |
| 1 | 오른손 | X/Y/Z 위치, Roll/Pitch/Yaw 방향 조절 |
| 2 | 오른손 손가락 | 엄지와 나머지 네 손가락의 접는 정도 조절 |
| 3 | 왼손 | X/Y/Z 위치, Roll/Pitch/Yaw 방향 조절 |
| 4 | 왼손 손가락 | 엄지와 나머지 네 손가락의 접는 정도 조절 |
| 5 | 전체 조인트 | 63개 조인트 관찰·직접 제어, `Pause`/`Resume`, `Reset` |
| 6 | 시뮬레이션 상태 | 오른손 IK 위치·회전 오차, 시간·루프 주파수 확인 |

장면에서 **왼쪽 드래그**는 시점 회전, **오른쪽 드래그**는 시점 이동, **마우스 휠**은 확대·축소입니다. `Esc`로 종료합니다.
패널 5의 `Pause`/`Resume`으로 물리 진행을 멈추거나 재개하고, `Reset`으로 초기 상태로 돌아갑니다.

### 손의 위치와 방향

| 입력 | 기준 | 단위 |
| --- | --- | --- |
| X / Y / Z | 월드 좌표계에서의 절대 위치 | m |
| Roll / Pitch / Yaw | 초기 손 방향에서 얼마나 회전할지 | 도(°) |

회전은 월드 고정축 X → Y → Z 순서로 적용합니다.

손에 표시된 좌표축은 **손의 실제 위치·방향을 따라가는 로컬 좌표계**입니다.
`R`은 오른손(패널 1), `L`은 왼손(패널 3)이며, 각 축의 길이는 15cm입니다.
X는 빨강, Y는 초록, Z는 파랑입니다. 위치 슬라이더의 X/Y/Z 기준은 월드 좌표계로 유지됩니다.

마지막으로 위치·방향을 조작한 손이 **활성 손**입니다.
활성 손이 바뀌면 반대 손의 그 시점 실제 XYZ 위치를 유지 목표로 저장합니다.
활성 손의 위치·방향과 반대 손의 XYZ 위치를 같은 우선순위로 추종하며, 반대 손의 방향은 제어 목표에 포함하지 않습니다.
두 목표를 동시에 만족할 수 없으면 오차를 절충하므로, 반대 손 위치가 반드시 고정되는 것은 아닙니다.

### 손가락 접고 펴기

패널 2·4의 `Thumb`는 엄지, `Four fingers`는 나머지 네 손가락을 조절합니다.
**0은 편 자세, 1은 설정된 접은 자세**이며, 중간값으로 접는 정도를 정합니다.
손가락을 조작해도 활성 손이나 팔 목표는 바뀌지 않습니다.

<details>
<summary>엄지의 방향과 접히는 방식</summary>

양손 모두 엄지 값이 0이면 손바닥 바깥쪽 법선(+X)을 향하며, 편 네 손가락(+Z)과 직각을 이룹니다.
값을 올리면 이 90° 기부 방향을 유지하면서 엄지 끝을 손바닥 쪽으로 더 굽힙니다.

</details>

### 캔 픽업 데모

`ffw-control --grasp-demo right` 또는 `ffw-control --grasp-demo left`로 실행합니다.
접근 → 캔 높이로 내리기 → 파지 → 손을 14cm 올리기 → 3초 유지 순서로 진행합니다.
완료 후 해당 손의 `Thumb`와 `Four fingers`를 모두 0으로 되돌리면 캔을 놓습니다.
진행 중 손의 위치·방향이나 픽업 중인 손의 손가락 목표를 바꾸거나, `Direct joints`를 켜면 시퀀스가 취소됩니다.
`Reset`은 데모를 종료하고 초기 배치로 돌아갑니다. 다시 실행하려면 위 명령으로 프로그램을 새로 시작하세요.

### 조인트 직접 제어

패널 5의 `Filter`와 목록 스크롤로 조인트를 찾을 수 있습니다.
항목에 마우스를 올리면 실제 위치·속도·구동기 힘·목표값을 표시합니다.

`Direct joints`를 켜면 패널 1~4의 입력이 멈추고 개별 조인트를 제어합니다.
바퀴 구동은 `v=`로 표시한 속도(rad/s), 나머지 조인트는 위치(rad 또는 m)를 입력합니다.

`Direct joints`를 끄면 현재 실제 자세에서 손의 위치·방향 제어를 재개합니다.
개별 조정한 손가락 자세는 다음 손가락 프리셋 입력까지 유지됩니다.

### 상태 표시 읽기

패널 6에는 IK 제어 중 다음 6개 항목을 표시합니다.
IK 오차는 **계산된 관절 목표로 얻은 손 자세**와 손 목표를 비교한 값이며, 실제 로봇 자세의 추종 오차와는 다릅니다.

| 표시 | 의미 |
| --- | --- |
| `ik_err (R)` | 오른손 기준 IK 위치 오차(mm). 조인트 직접 제어 중에는 `N/A` |
| `Right IK rotation` | 오른손이 활성 손일 때의 IK 회전 오차(°) |
| `sim_time` | 시뮬레이션 시간(s) |
| `wall_time` | 실행 후 실제 경과 시간(s) |
| `loop_freq` | 실제 루프 주파수(Hz) |
| `loop_freq (ema)` | 루프 주파수의 지수 이동 평균(Hz) |

오른손이 반대 손이면 방향 오차 대신 `Right rotation: free`를 표시합니다.
조인트 직접 제어 중에는 회전 오차 줄을 숨기고 나머지 5개 항목만 표시합니다.
`loop_freq`는 제어 루프의 실제 실행 간격으로 계산하므로, 설정된 제어 주파수 50Hz와 다를 수 있습니다.

## 화면 없이 실행하기

`--headless`를 붙이면 GUI 없이 계산하고, 종료 시 오차와 상태를 **JSON으로 출력**합니다.
`--seconds`는 실제 대기 시간이 아닌 **시뮬레이션 시간**이며, 실시간 대기 없이 계산합니다.
기본값은 5초, 픽업 데모는 25초입니다.

가상환경을 활성화한 뒤 프로젝트 루트에서 실행하세요.

```bash
# 초기 자세를 3초간 유지
ffw-control --headless --seconds 3

# 오른손: 초기 위치에서 X +4cm, Z +3cm, Roll +5°
ffw-control --headless --seconds 4 --offset 0.04 0 0.03 --rpy 5 0 0

# 양팔과 리프트를 함께 사용하는 큰 이동: 오른손 X +20cm
ffw-control --headless --seconds 8 --offset 0.20 0 0

# 왼손: 초기 위치에서 X +3cm, Z +3cm
ffw-control --headless --seconds 4 --side left --offset 0.03 0 0.03
```

GUI 위치 슬라이더와 달리, `--offset`은 초기 손 위치에 더할 **이동량(m)**입니다.
`--side`를 생략하면 오른손에 적용합니다. `--rpy`는 Roll/Pitch/Yaw 순서이며 단위는 도(°)입니다.

손가락, 조인트, 픽업 데모는 다음과 같이 실행합니다.

```bash
# 양손 손가락: 각 손의 엄지, 나머지 네 손가락 순서 (0~1)
ffw-control --headless --seconds 3 --right-fingers 1 1 --left-fingers 0.6 0.8

# 조인트 직접 제어: --joint를 반복해서 여러 조인트 지정
ffw-control --headless --seconds 3 --joint head_joint1=0.2 --joint finger_r_joint6=0.6

# 캔 파지·들기·유지 (기본 25초)
ffw-control --headless --grasp-demo right
ffw-control --headless --grasp-demo left
```

`--joint`는 손 위치·방향 및 손가락 프리셋 입력과 함께 사용할 수 없습니다.
`--grasp-demo`도 수동 제어 입력과 함께 사용할 수 없습니다.
전체 옵션은 `ffw-control --help`에서 확인할 수 있습니다.

### JSON 결과 읽기

패널 6에 없는 실제 추종 오차, 리프트 위치, 캔 상태와 픽업 진행 상황은 화면 없는 실행의 JSON에서 확인합니다.

| 항목 | 의미 |
| --- | --- |
| `position_error_m` / `orientation_error_deg` | 실제 활성 손의 위치·방향 추종 오차(m / °) |
| `ik_position_error_m` / `ik_orientation_error_deg` | 계산된 관절 목표의 활성 손 위치·방향 오차(m / °) |
| `held_position_error_m` / `ik_held_position_error_m` | 실제 자세 / 계산된 관절 목표의 반대 손 XYZ 위치 오차 크기(m) |
| `ik_err` / `right_ik_orientation_error_deg` | 패널 6과 같은 오른손 기준 IK 오차. JSON의 위치 단위는 m |
| `lift_m` | 실제 리프트 조인트 위치(m) |
| `objects` | 양쪽 캔의 위치·접촉·들림 상태 |
| `grasp_demo` | 데모의 대상 손, 진행 단계(`phase`), 설명(`message`). 데모를 실행하지 않으면 `null` |
| `mode` / `status` | 계산 모드와 추종 상태 |
| `saturated_actuators` | 마지막 물리 스텝에서 양팔·리프트 구동기 명령이 힘·토크 제한으로 잘린 개수. 누적 횟수가 아님 |

오차 항목은 조인트 직접 제어 중 `null`이며, `right_ik_orientation_error_deg`는 오른손이 반대 손일 때도 `null`입니다.
`status`가 `IK reached / settling`이면 계산된 관절 목표가 허용 오차 안에 있고, `Reached`이면 실제 로봇도 허용 오차 안에 있습니다.
`Tracking / limited`는 아직 추종 중이거나 제약 때문에 오차가 남은 상태입니다.
`mode`가 `STOP`이면 QP 계산 실패로 마지막 유효 관절 목표를 유지합니다.
화면 없는 실행은 `STOP` 또는 픽업 단계 `FAILED`일 때 JSON을 출력한 뒤 종료 코드 1을 반환합니다.
종료 코드 0만으로 목표 도달이나 픽업 완료를 뜻하지는 않으므로, `status`와 `grasp_demo.phase`를 함께 확인하세요.

### GUI 화면 저장

다음 명령은 GUI를 120프레임 동안 실행하고 마지막 화면을 저장한 뒤 종료합니다. 데스크톱 환경이 필요합니다.

```bash
ffw-control --frames 120 --screenshot outputs/panels.png
```

저장 경로의 폴더는 자동으로 생성됩니다. `outputs/`는 Git에서 제외됩니다.

## 설정과 코드

### 설정을 바꾸려면

| 파일 | 바꿀 수 있는 내용 |
| --- | --- |
| [configs/robot.json](configs/robot.json) | 장면 경로, IK용 팔·공유 리프트 조인트, 손 기준점 매핑 |
| [configs/control.json](configs/control.json) | 초기 자세, 제어 주기, 속도·가속도 제한, 허용 오차, 제어 게인 |
| [configs/fingers.json](configs/fingers.json) | 손가락 펴기/접기 자세, 명령 속도 제한 |
| [scenes/grasp.xml](scenes/grasp.xml) | 작업대·캔 배치와 물리 접촉 설정 |
| [robotis_ffw/ffw_sh5.xml](robotis_ffw/ffw_sh5.xml) | 관절의 물리 감쇠·마찰·관성, 원본 서보 게인과 구동기 힘·토크 제한 |
| [src/ffw_control/control/dynamics/servo.py](src/ffw_control/control/dynamics/servo.py) | 런타임 PD 제어, 양팔 `Kd` 배율과 힘 제한 적용 |

현재 주요 설정은 다음과 같습니다. 설정 파일을 바꾸면 프로그램을 재시작하거나 `Reset`으로 모델을 다시 생성하세요.

| 설정 | 현재 값 | 적용 대상 |
| --- | --- | --- |
| `arm_velocity_limit` | 3.15 rad/s | 양팔 관절 목표 속도 |
| `lift_velocity_limit` | 0.1575 m/s | 리프트 목표 속도 |
| `arm_acceleration_limit` | 3.0 rad/s² | IK에서 양팔 관절 명령 가속도 |
| `lift_acceleration_limit` | 0.18 m/s² | IK에서 리프트 명령 가속도 |
| `hand_linear_velocity_limit` | 0.45 m/s | IK에서 양손 위치 오차를 보정하는 요구 속도 |
| `hand_angular_velocity_limit` | 2.25 rad/s | IK에서 활성 손 방향 오차를 보정하는 요구 속도 |
| `task_gain` | 4.0 s⁻¹ | IK 오차를 요구 속도로 변환하는 비례 게인 |

IK는 계산용 기준 자세에서 `요구 속도 = task_gain × 목표 오차`를 구한 뒤 손 속도 상한을 적용하고,
관절 위치·속도·가속도·제동 거리 제약 안에서 관절 목표를 계산합니다.
`Direct joints`에서도 팔·리프트의 목표 속도 제한은 적용하지만, IK의 가속도·제동 거리 제약은 적용하지 않습니다.
이 제한들은 목표 생성에 적용되며 실제 로봇의 속도·가속도 상한을 보장하지 않습니다.
짧은 이동에서는 가속도 제한이나 작은 목표 오차 때문에 최고 속도에 도달하지 않을 수 있습니다.

양팔의 `Kp`는 XML 원본 값을 사용하고, `Kd`는 원본 서보에서 계산한 값의 50%를 사용합니다.
리프트는 별도로 `lift_position_gain=100000`, `lift_velocity_gain=600`을 사용합니다.
관절의 물리 `damping`과 구동기 힘·토크 제한은 원본 값을 유지합니다.
팔 토크 제한은 관절 1·2가 ±61.4 Nm, 3~6이 ±31.7 Nm, 7이 ±5.1 Nm입니다.
토크 제한을 높이는 것은 이미 포화된 명령에만 영향을 주며, 접촉 시 더 큰 힘을 낼 수 있습니다.

패키지와 실행 명령은 [pyproject.toml](pyproject.toml), 의존성과 버전은 [requirements.txt](requirements.txt)에서 관리합니다.

`position_tolerance`와 `orientation_tolerance_degrees`는 활성 손의 위치·방향 도달 판정 기준입니다.
`held_position_tolerance`는 반대 손의 XYZ 위치 오차 크기에 대한 도달 판정 기준이며, 기본값은 2mm입니다.
이 허용 오차들은 하드 제약이 아니라 상태 표시를 위한 기준입니다.
`initial_lift`는 시작과 `Reset` 시의 리프트 조인트 위치이며, 기본값 0.0은 최상단입니다.
기본 제어 주기는 0.02초(50Hz), 물리 계산 간격은 0.002초(500Hz)입니다.

### 코드를 읽으려면

```text
src/ffw_control/
├── app.py                  # GUI·화면 없는 실행의 진입점
├── robots/model.py         # 고정 베이스 모델, 손 기준점, Jacobian
├── control/
│   ├── fingers.py          # 엄지·네 손가락 독립 제어
│   ├── joints.py           # 전체 조인트 조회·직접 제어
│   ├── hierarchy/qp.py     # 제약 QP와 우선순위에 따른 계산
│   ├── ik/differential.py  # 양손 목표를 같은 우선순위로 푸는 미분 IK
│   └── dynamics/servo.py   # 힘 제한 PD, 중력·코리올리·원심력 보상
├── simulation/
│   ├── session.py          # 양손 목표, 제어 모드, 물리 진행
│   ├── timing.py           # 실제 루프 주파수·EMA
│   ├── grasp.py            # 픽업 시퀀스
│   └── objects.py          # 캔 접촉·들림 관측
└── ui/window.py            # 슬라이더와 MuJoCo 렌더링
```

제어 흐름은 **손 목표 입력 → 관절 목표 계산 → 물리 시뮬레이션에서 추종** 순서입니다.
관절 목표는 공통 제약을 적용한 미분 역기구학(IK)과 계층적 이차계획법(HQP)으로 계산합니다.
양팔 14관절과 공유 리프트 1축의 속도를 항상 함께 계산하며, 팔 우선 모드나 리프트 사용 최소화 단계는 없습니다.

| 구분 | 내용 |
| --- | --- |
| 공통 하드 제약 | 관절 위치·속도·가속도 제한과 관절 한계 앞에서의 제동 거리 제한 |
| HQP 1순위 | 활성 손 위치·방향 6차원 + 반대 손 XYZ 위치 3차원의 오차를 함께 최소화 |
| HQP 2순위 | 정규화된 관절 속도 최소화, 이전 속도와의 차이 최소화, 팔의 초기 자세 복귀 선호 |

양손 위치 항의 가중치는 같고, 활성 손의 회전 오차는 0.2를 곱해 위치 오차와 함께 계산합니다.
2순위는 1순위에서 달성한 결과를 보존하는 여유 자유도 안에서 계산합니다.
반대 손의 방향 유지 목표와 위치·높이 하드 제약은 없으며, 리프트의 초기 높이 복귀 목표도 없습니다.
위치 오차는 손바닥 기준점의 월드 좌표로 계산하고, 방향 오차는 SO(3) 회전 벡터로 계산합니다.
계산된 관절 목표는 힘 제한을 적용한 PD 제어와 중력·코리올리·원심력 보상으로 추종합니다.

## 문제 해결

### GUI 창이 열리지 않을 때

창을 열 수 있는 데스크톱 터미널에서 실행했는지 확인하세요.
화면 없이 모델과 제어 동작을 확인하려면 `ffw-control --headless --seconds 3`을 사용합니다.

### `ffw-control` 명령을 찾을 수 없을 때

프로젝트 루트에서 `source .venv/bin/activate`로 가상환경을 활성화하세요.
아직 설치하지 않았다면 `python -m pip install -e .`을 실행합니다.

### ROS를 불러온 터미널에서 의존성을 점검할 때

ROS의 `PYTHONPATH`는 가상환경에도 상속됩니다.
프로젝트 가상환경의 의존성만 점검하려면 다음 명령을 사용하세요.

```bash
env -u PYTHONPATH .venv/bin/python -m pip check
```

## 로봇 모델 라이선스

SH5 원본 XML과 필수 메시는 `robotis_ffw/`에 있습니다.
로봇 모델의 라이선스는 [robotis_ffw/LICENSE](robotis_ffw/LICENSE)를 확인하세요.
