# FFW-SH5 처음 실행하기

MuJoCo는 로봇의 움직임과 충돌을 계산하고 화면에 보여주는 시뮬레이터입니다.
이 예제는 가상 로봇의 머리와 양팔을 움직입니다.

## 1. 설치 (최초 한 번)

Ubuntu/Linux 터미널에서 저장소 폴더로 이동한 뒤 실행합니다.
`.venv`는 이 프로젝트에서 사용할 Python과 패키지를 따로 보관하는 폴더입니다.

```bash
cd /home/hwangjeongui/robotis_mujoco_menagerie
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-ffw.txt
```

이미 설치가 끝났다면 다음 단계부터 시작하면 됩니다.
이 예제에서 확인한 MuJoCo 버전은 `3.3.7`입니다.

## 2. 자동으로 움직여 보기

```bash
.venv/bin/python run_ffw_sh5.py
```

3D 창에서 로봇이 고개를 좌우로 돌리고 양쪽 팔꿈치를 천천히 굽혔다 폅니다.
Space 키로 멈추거나 다시 움직이고, R 키로 초기 상태로 돌아갑니다.
창을 닫으면 종료됩니다. 터미널에서 Ctrl+C를 눌러도 됩니다.

마우스 왼쪽 버튼을 누른 채 드래그하면 시점을 회전하고,
오른쪽 버튼으로 드래그하면 시점을 이동합니다. 휠로 확대·축소합니다.
키보드 조작은 3D 창을 클릭한 뒤 사용하세요.

## 3. 내가 직접 움직여 보기

자동 실행 창을 닫은 뒤 실행합니다.

```bash
.venv/bin/python run_ffw_sh5.py --manual
```

오른쪽 패널의 **Control**을 펼치고 슬라이더를 조금씩 움직입니다.
오른쪽 패널이 보이지 않으면 Shift+Tab을 누릅니다.
일시정지 상태에서는 Space를 눌러 시뮬레이션을 재개해야 목표를 따라 움직입니다.

처음에는 다음 순서로 조작해 보세요.

| 슬라이더 이름 | 움직이는 부분 | 처음 넣어볼 값 |
| --- | --- | --- |
| `head_joint2` | 고개 좌우 회전 | `0.2` |
| `head_joint1` | 고개 위아래 회전 | `0.2` |
| `arm_l_joint4` | 왼쪽 팔꿈치 | `-0.5` |
| `arm_r_joint4` | 오른쪽 팔꿈치 | `-0.5` |
| `finger_l_joint6` | 왼손 검지 굽힘 관절 | `0.4` |
| `lift_joint` | 상체 높이 | `-0.1` |

팔·머리·손가락의 값은 목표 각도이며 단위는 라디안입니다.
`0.2 rad`는 약 11도, `0.5 rad`는 약 29도입니다.
`lift_joint`는 미터 단위로, 음수 방향으로 상체를 내립니다.
`*_wheel_steer`는 바퀴 방향의 목표 각도이고,
`*_wheel_drive`는 바퀴 회전 속도(rad/s)입니다.
처음 관절을 익힐 때는 바퀴 속도를 `0`으로 두세요.

## 4. 오른손 위치와 방향을 슬라이더로 조정하기

`proj.py`는 X·Y·Z(m), Roll·Pitch·Yaw(도) 슬라이더로 오른손 손바닥의 목표 자세를
지정합니다. 오른팔 7개 관절과 상체 승강 관절을 역기구학으로 함께 제어합니다.

```bash
.venv-practice/bin/python -m pip install -r requirements-proj.txt
.venv-practice/bin/python proj.py
```

`.venv-practice` 환경이 없다면 먼저 `python3 -m venv .venv-practice`로 생성하세요.
위치는 로봇 하부 기준 좌표이며, 회전은 시작 손 방향에 대한 변화량입니다.
가상환경 설정, UI 구성, 역기구학과 제어 원리, 검증 결과는
[오른손 제어 구현 설명](../PROJ_IK_GUIDE.md)에 정리했습니다.

## 파일의 역할

| 파일 | 역할 |
| --- | --- |
| `scene_ffw_sh5.xml` | 로봇과 바닥, 조명을 합친 장면. 실행 예제가 읽는 파일 |
| `ffw_sh5.xml` | 관절, 모터, 질량, 충돌 등 로봇의 구조와 물리 설정 |
| `assets/` | 로봇 각 부품의 3D 모양 |
| `../run_ffw_sh5.py` | 장면을 불러오고 모터에 목표값을 보내는 실행 예제 |

처음에는 XML을 수정할 필요가 없습니다. 실행 예제가 시작할 때 바퀴를 바닥에
맞추고 양팔을 몸통에서 약간 벌립니다. 원본 XML 파일은 수정하지 않습니다.
이 예제는 관절 제어를 체험하는 용도이며 이동이나 물체 잡기 동작은 포함하지 않습니다.

## 화면이 뜨지 않을 때

GUI 창을 열 수 있는 Linux 데스크톱의 터미널에서 실행하세요.
실행 예제는 현재 데스크톱과의 호환성을 위해 Linux에서 X11 창과 EGL 렌더링을
사용합니다. 시스템의 그래픽 드라이버 설정은 바꾸지 않습니다.
화면 없이 모델과 물리 계산만 확인하려면 다음을 실행합니다.

```bash
.venv/bin/python run_ffw_sh5.py --headless --seconds 5
```

자동 실행을 정해진 시간 뒤 종료하려면 `--seconds 10`처럼 지정합니다.
이 시간은 시뮬레이션 시간 기준입니다.

MuJoCo 설치와 뷰어 사용법: <https://mujoco.readthedocs.io/en/stable/python.html>
