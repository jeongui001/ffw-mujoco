# FFW-SH5 양손 제어 프로젝트

MuJoCo에서 슬라이더(스크롤 바)로 FFW-SH5의 왼손과 오른손을 움직이는 프로젝트입니다.
현재 단계는 **가상환경 및 폴더 구조 준비**입니다. 양손 제어 UI와 제어 알고리즘은 아직 구현하지 않았습니다.

## 환경 준비

프로젝트 루트에서 실행합니다. Python 3.10 기준이며 가상환경 폴더 이름은 `requirements-ffw`입니다.
패키지 목록인 `requirements-ffw.txt`와 구분합니다.

```bash
python3 -m venv --prompt requirements-ffw requirements-ffw
source requirements-ffw/bin/activate
python -m pip install -r requirements-ffw.txt
python -m pip install -e .
python scripts/check_model.py
```

이미 생성한 환경은 `source requirements-ffw/bin/activate`로 활성화하면 됩니다.
종료는 `deactivate`입니다. VS Code에서는 `requirements-ffw/bin/python`을 인터프리터로 선택합니다.

ROS를 불러온 터미널에서는 `PYTHONPATH`가 가상환경에도 상속됩니다.
프로젝트 패키지만 대상으로 의존성을 점검하려면 다음을 실행합니다.

```bash
env -u PYTHONPATH requirements-ffw/bin/python -m pip check
```

`check_model.py`는 창을 열지 않고 모델을 로드하고 초기 기구학을 계산하여 양팔 관절·액추에이터와 손 기준 body를 확인합니다.
동역학 안정성이나 제어 성능 검증은 포함하지 않습니다.

## 폴더 구조

```text
mujoco_proj/
├── requirements-ffw/          # 로컬 가상환경 (Git 제외)
├── requirements-ffw.txt       # 기존 패키지 버전 목록
├── pyproject.toml            # Python 패키지 설정; 의존성은 위 목록 참조
├── robotis_ffw/              # 기존 로봇 XML 및 메쉬
├── configs/
│   └── robot.json            # 장면 경로, 양팔 관절, 손 기준 body 매핑
├── scenes/                   # 향후 프로젝트 전용 장면 및 모델 변형
├── src/ffw_control/
│   ├── ui/                   # 좌·우 목표 슬라이더, 초기화, 상태 표시
│   ├── robots/               # 모델 로딩, 이름→ID 매핑, 좌표계
│   ├── simulation/           # 시뮬레이션 루프, 상태, 초기화, 뷰어
│   ├── control/
│   │   ├── ik/               # 역기구학
│   │   ├── dynamics/         # 중력 보상, 토크/구동기 변환
│   │   └── hierarchy/        # 양손 작업, 자세, 제한조건의 우선순위
│   └── evaluation/           # 추종 오차·제약 위반·보상 함수
├── scripts/
│   └── check_model.py        # 화면 없는 모델 로딩 점검
├── tests/                    # 구현 후 추가할 자동 테스트
├── experiments/              # 재현 가능한 실험 설정 및 실행 조건
├── outputs/                  # 생성 로그, 그래프, 영상 (Git 제외)
└── docs/
    ├── functional_spec.md    # 사용자 기능, 패널 구성, 장면 요구사항
    ├── panel1_control_design.md # 오른손 IK 후보 및 제어 추천안
    └── architecture.md       # 모듈 책임, 데이터 흐름, 후속 설계 항목
```

각 코드 폴더는 현재 패키지 골격만 갖고 있습니다. 기존 `robotis_ffw/README.md`는 이전 예제의 안내이며,
현재 저장소의 환경 및 실행 안내는 이 문서를 기준으로 합니다.

프로젝트 Markdown 문서는 `docs/`에 모읍니다.
예정된 기능은 [사용자 기능 명세](docs/functional_spec.md),
설계 의도와 다음 단계는 [구조 설계 문서](docs/architecture.md)를 참고하세요.
패널 1의 IK 후보와 제어 추천안은 [패널 1 알고리즘 검토](docs/panel1_control_design.md)에 정리합니다.
