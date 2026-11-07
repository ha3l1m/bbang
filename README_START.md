# 붕어빵 로봇 — RViz 시험 / 실로봇 도구 (클리커 3개 + 사다리 뚜껑 틀 + 소스통 + DRL 그리퍼)

## 1) 압축을 레포 루트(boong/)에 덮어 풀기
```
cd ~/boong            # 레포 루트 (ros2_ws/ 가 있는 곳)
unzip -o ~/Downloads/ros_tools_clicker.zip
```
- 덮어쓰는 파일: `ros2_ws/src/boong_robot/boong_robot/{motions,poses,gripper,drl_gripper}.py`, `test/test_motions.py`
- 새 파일: `rviz_demo/`, `real_robot/`, `config/poses.clicker.example.yaml` (**기존 poses.yaml 은 건드리지 않음**)

## 2) 테스트 (ROS 없이, 로봇 없이)
```
cd ros2_ws/src/boong_robot && PYTHONPATH=. python3 -m pytest -q test      # 28개
cd ~/boong/rviz_demo && PYTHONPATH=../ros2_ws/src/boong_robot:. python3 -m pytest -q tests   # 11개
```

## 3) RViz 로 보기 (Ubuntu 22.04=Humble / 24.04=Jazzy, ROS + RViz2 설치돼 있어야 함)
```
cd ~/boong/rviz_demo
./setup_ubuntu.sh        # 처음 한 번: 필요한 ROS 패키지 확인/설치 (sudo)
./run_rviz_demo.sh --poses ../ros2_ws/src/boong_robot/config/poses.clicker.example.yaml --count 3
```
RViz 가 열리면 `/boong/viz` (MarkerArray) 가 자동으로 보입니다. 옵션: `--speed 3`, `--step flip_mold`(뚜껑 닫고 열기만), `--loop`, `--table 120,1080,275,275`
- 확인할 것: 소스통 짜기(3칸) → 뚜껑 틀이 가운데 고리 봉 둘레로 반원 → 굽는 동안 대기 → 다시 열기 → 봉투
- **툴 회전 방향(`roll_sign`)**: 뚜껑이 반대로 돌거나 봉이 뒤틀려 보이면 `poses.clicker.example.yaml` 의 `frame.roll_sign` 을 `1` 로 바꿔 보세요.

## 4) 실로봇 전 (Jazzy + doosan-robot2)
`real_robot/README.md` 참고. 순서: `setup_doosan_jazzy.sh` → `bringup_virtual.sh`(에뮬레이터) → `run_real.py --gripper mock` → 마지막에 `--gripper drl`.
좌표·고리 간격(60mm)·테이블 크기는 모두 **예시/추정값** → 현장 실측 후 교체.
