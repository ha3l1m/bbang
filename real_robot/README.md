# 실제 로봇 실행 (E0509 · 컨트롤러 110.120.1.64 · ROS 2 Jazzy / Ubuntu 24.04)

레포 맨 위에 `rviz_demo/` 옆에 `real_robot/` 로 두세요.

## 0. 사진으로 확인한 것
- 로봇: Doosan **E0509** (Reach 900mm, Max payload 5kg). 베이스는 구멍 뚫린 작업대 판에 고정.
- 그리퍼 라벨: **RH-P12-RN** (끝에 `-A` 없음). 팀원 레포는 `RH-P12-RN-A` 용이라 **그대로 쓸 수 있는지 팀원에게 확인** 필요 (통신/제어 방식이 다를 수 있음).
- 컨트롤러 DRCF **2.x** → 두산 MoveIt 은 쓰지 않음(MoveIt 은 2.12+ 필요). 우리는 `DSR_ROBOT2`(movej/movel)만 쓰므로 영향 없음. 빌드할 때 `-DDRCF_VER=3` 은 **넣지 않음**.

## 1. 한 번만 설치
```bash
cd boong/real_robot
./setup_doosan_jazzy.sh      # doosan-robot2 (jazzy 브랜치) 빌드, 5~15분
```
끝에 안내되는 두 줄(`source ...install/setup.bash`, `PYTHONPATH=...imp`)을 `~/.bashrc` 에 넣고 새 터미널을 여세요.
(`ModuleNotFoundError: No module named 'DR_init'` 가 나면 PYTHONPATH 줄이 빠진 것)

## 2. 네트워크
- 노트북 유선 IP 를 같은 대역으로: 예) `110.120.1.100/24` (로봇은 110.120.1.64)
- `ping 110.120.1.64` 가 되어야 함
- 티치펜던트: 서보 ON, **원격 제어(Remote) 허용**, 속도 낮게

## 2.5 real 전에: RViz 로 먼저 (순서)
1. `rviz_demo/run_rviz_demo.sh` — 동작 순서·테이블 배치 확인 (ROS 만 있으면 됨, 근사 팔)
2. `./bringup_virtual.sh` + `python3 run_real.py ...` — **진짜 E0509 모델 + 두산 에뮬레이터**로 real 과 똑같은 명령 실행 (Docker 필요, 처음 한 번 `sudo ./install_emulator.sh`)
3. 둘 다 이상 없으면 아래 real

## 3. 실행 순서
```bash
# 터미널 1: 로봇 연결 + RViz (RViz 에서 자세가 실제와 같으면 연결 성공)
./bringup_real.sh                      # 기본 IP 110.120.1.64, 다른 IP: ROBOT_IP=... ./bringup_real.sh

# 터미널 2: 연결 확인 — J6 만 ±10° (비상정지 손에!)
python3 run_real.py --test-j6

# 동작 하나씩 (그리퍼는 mock = 안 움직임, 속도 50mm/s). 모든 명령이 먼저 출력되고 yes 를 쳐야 움직임
python3 run_real.py --step pour_batter
python3 run_real.py --step flip_mold
# ... 좌표가 맞으면 전체
python3 run_real.py --flavor redbean
# 그리퍼까지: 팀 그리퍼 bringup 이 떠 있을 때만
python3 run_real.py --step pour_batter --gripper ros_service
```
- `run_real.py` 는 실행 전에 **드라이런**으로 z < 30mm, 베이스에서 850mm 초과 좌표를 걸러서 거부해요 (`plan_check.py`).
- 좌표 수정은 `ros2_ws/src/boong_robot/config/poses.yaml` 만. 고치고 다시 실행하면 바로 반영.

## 4. 현장 보정 체크리스트 (토)
1. **TCP/툴 무게 설정** (티치펜던트): 그리퍼 무게(+통 포함). TCP 를 손가락 끝에 두면 poses.yaml 의 z 가 '손가락 끝 높이'가 되어 다루기 쉬워요. TCP 를 안 정하면 좌표는 플랜지 기준이라 모든 z 가 달라져요.
2. 티치펜던트로 손으로 끌어서(다이렉트 티칭) 위치를 잡고 값을 poses.yaml 에 옮기기: 통 3개, mold_dispense, mold_handle, bread_pick, bag_drop
3. mold_dispense z: 통을 뒤집었을 때 노즐이 틀 위 1~2cm (통 길이의 절반이 손 아래로 내려옴)
4. 그리퍼: 통 지름이 그리퍼 벌어지는 폭 안인지, hold/squeeze 값(0~700) 보정
5. 첫 동작은 전부 빈 통/빈 틀로

## 5. 막히면
| 증상 | 확인 |
|---|---|
| ping 실패 | 랜선, 노트북 IP 대역, 방화벽 |
| bringup 은 뜨는데 RViz 에서 안 움직임 | 서보 ON? 원격 제어? 컨트롤러 모드 Autonomous |
| `DR_init` 못 찾음 | PYTHONPATH (위 1번) |
| `DSR_ROBOT2` import 에러 | 노드/DR_init 설정 '뒤에' import 해야 함 (run_real.py 는 그렇게 되어 있음) |
| MoveIt 관련 에러 | 안 쓰니 무시 (컨트롤러 2.0 은 MoveIt 불가) |

참고: [Doosan ROS 2 Jazzy 매뉴얼](https://doosanrobotics.github.io/doosan-robotics-ros-manual/jazzy/installation.html), [DR_init 문제](https://doosanrobotics.github.io/doosan-robotics-ros-manual/jazzy/faq/dr_init.html)
