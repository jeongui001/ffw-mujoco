# 자동 테스트

```bash
source requirements-ffw/bin/activate
python -m unittest discover -s tests -v
```

`test_control.py`는 상위 작업 보존, 제한 내 IK, SO(3) 오차, 양손 위치·회전 추종,
팔 우선 정책, 공유 리프트와 반대 손 높이 유지, 좌우 전환, 명령 속도·가속도 및 힘 제한,
도달 불가 입력, 잘못된 입력과 오래된 백그라운드 계획의 폐기를 검증합니다.
화면을 열지 않고 실제 MuJoCo 모델과 물리 진행을 사용합니다.

원본 모델 로딩과 이름 매핑만 확인하려면 `python scripts/check_model.py`를 실행합니다.
