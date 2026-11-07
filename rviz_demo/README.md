# 붕봇 RViz 데모 (Ubuntu 24.04 + ROS 2 Jazzy, 22.04 + Humble 도 가능)

로봇·두산 패키지·Docker 없이, `motions.py`가 내보내는 동작(movel / J6 회전 / 그리퍼)을 **RViz에서 눈으로 확인**하는 도구예요.
통을 집어 → 틀 위에서 뒤집어 → 눌러 짜고 → 틀을 뒤집고 → 봉투에 담는 순서가 그대로 재생돼요.
`ros2_ws/src/boong_robot/config/poses.yaml` 좌표를 바꾸면 바로 반영돼요.

## 놓는 위치
레포 맨 위(`boong/`)에 이 폴더를 `rviz_demo/` 로 두세요.
```
boong/
├─ ros2_ws/src/boong_robot/...
└─ rviz_demo/   ← 여기
```

## 실행
```bash
cd boong/rviz_demo
./setup_ubuntu.sh          # 처음 한 번만 (ROS 2 + rviz2 가 이미 있으면 금방 끝남)
./run_rviz_demo.sh         # 단팥 1개 (RViz 창이 자동으로 뜸)
```
옵션
| 명령 | 설명 |
|---|---|
| `./run_rviz_demo.sh --flavor cream --count 2` | 슈크림 2개 |
| `./run_rviz_demo.sh --speed 1` | poses.yaml 속도 그대로 (기본은 3배) |
| `./run_rviz_demo.sh --step pour_batter` | 동작 하나만 (`add_filling` `cover_filling` `wait_bake` `flip_mold` `pick_to_bag`) |
| `./run_rviz_demo.sh --loop` | 계속 반복 |
| `./run_rviz_demo.sh --table 120,1080,275,275` | 테이블 크기(mm): 베이스 중심에서 뒤/앞/왼쪽/오른쪽 가장자리까지. 줄자로 잰 값으로 바꾸세요 |

터미널에 `[batter] 반죽 짜는 중` 같은 단계가 찍혀요 (웹앱 단계 이름과 같음).

## RViz에서 보이는 것
- 파란 선 = 근사 팔 / 빨간 점 = 그리퍼 손가락 (짜면 더 모임) / 가운데 통이 잡힌 통
- **통이 거꾸로 뒤집혀(노즐 아래) 틀 위에서 눌리며 반죽이 나오고**, 틀 위에 덩어리가 커져요
- 틀 손잡이를 잡아 180° 뒤집는 장면, 붕어빵이 봉투로 가는 장면
- 화면에 `out of reach!` 가 뜨면 그 좌표는 팔이 못 닿는 곳이에요 (근사 모델 기준)

## 테이블 배치 확인
RViz 에 연한 회색 판(테이블)이 같이 보여요. 통·봉투·툴 경로가 판 밖으로 나가면 터미널에 `⚠️ 배치:` 가 찍히고 화면에 `LAYOUT WARNING` 이 떠요. 현재 예시 좌표는 폭 58cm 테이블에 안 맞아서 경고가 나는 게 정상이에요 → poses.yaml 을 안쪽으로 옮긴 뒤 경고가 사라질 때까지 반복.

## 안 보일 때
- RViz 왼쪽 `Fixed Frame` = `base_link`, MarkerArray 토픽 = `/boong/viz` 인지 확인
- 다른 터미널에서 `source /opt/ros/$ROS_DISTRO/setup.bash && ros2 topic hz /boong/viz` → 30Hz 근처면 정상
- 가상머신/WSL 에서 RViz 가 안 뜨면 GPU 가속 문제예요 → 실제 우분투(또는 WSLg)에서 실행
- `rviz_demo/` 위치가 다르면 `BOONG_REPO=/레포/경로 ./run_rviz_demo.sh`

## 한계 (중요)
- 팔은 **근사 모델**(어깨155·위팔409·아래팔367·툴160mm)이라 실제 E0509 모양/관절 한계/충돌은 안 봐요.
  → 실제 모델로 보려면 Docker 에뮬레이터 경로(`./robot.sh bringup`)를 쓰세요.
- 통 크기는 높이 180·폭 70mm 가정(`arm_model.py` 맨 위). 실제 통과 다르면 고치세요.
- 짜는 세기/시간은 모양만 보여줘요. 실제 짜지는 양은 현장에서 보정.

## 코드 검증 (ROS 없이)
```bash
cd rviz_demo && python3 -m pytest -q tests
```
