# 클리커 + 커튼봉 + 소스통 설계 업데이트 (overlay)

## 1) 레포에 덮어쓸 것 (repo_update/)
- `motions.py`, `poses.py`, `test_motions.py` → `ros2_ws/src/boong_robot/...` 에 덮어쓰기
- `poses.clicker.example.yaml` 은 **예시**. 본인 `poses.yaml`(보정한 좌표)은 덮어쓰지 말고 아래 키만 추가:
  ```yaml
  flip_lift_mm: 0          # 봉은 들어올리지 않고 J6만 회전
  cells: {count: 3, pitch_mm: 100}   # 클리커 개수 / 간격(y)
  ```
- 테스트: `cd ros2_ws/src/boong_robot && PYTHONPATH=. python3 -m pytest -q test`

## 2) 동작
소스통(반죽/앙금)을 옆에서 잡고 → J6 180°로 노즐 아래 → 각 클리커 칸(`cells`) 위에서 그리퍼로 짜기 → 통 반납 →
커튼봉 끝(`mold_handle`)을 잡고 J6 +180° (모든 뚜껑 닫힘) → 굽기 대기 → J6 −180° (열림) → `bread_pick`.
`make_batch(flavor, n)` : 1 ≤ n ≤ cells.count.

## 3) RViz 확인
```
cd rviz_demo
./run_rviz_demo.sh --poses ../repo_update/ros2_ws/src/boong_robot/config/poses.clicker.example.yaml --count 3
```
## 4) 실로봇 전 dry-run
```
cd real_robot
python3 run_real.py --poses <poses.yaml> --cells 3 --flavor redbean   # 기본 gripper mock, 확인 후 yes
```

## 5) 현장 확인 사항 (추정값)
- 소스통 ≈ 210×65 mm(400 ml) 추정 / 클리커 ≈ 9×5 cm 추정 → 실측 필요
- 커튼봉은 클리커 힌지선과 몇 mm 이내 동축, 받침(saddle) 위, 뚜껑에 고정(발포 테이프로 유격 흡수)
- 클리커 받침대(riser)로 봉 축 z≈85 mm
- 봉 끝 접근점이 테이블 가장자리 근처 → 손목이 테이블 밖으로 나감, 충돌 확인
- 매끈한 봉은 그리퍼가 미끄러질 수 있음 → 고무/테이프
- TCP/페이로드는 티치펜던트에서 설정, 좌표는 전부 현장 보정
- 그리퍼 RH-P12-RN vs 팀 레포(RN-A, Humble) 호환성 미확인

## 6) 그리퍼: DRL 방식 추가 (공식 doosan-robot2 만 사용)
`repo_update/.../drl_gripper.py` + `gripper.py` 가 `type: drl` 을 지원한다 (사용자가 검증한 gripper_drl.py 와 프레임·DRL 코드가 바이트 단위로 동일함을 확인).
- poses.yaml: `gripper: {type: drl, namespace: /dsr01, move_wait_sec: 1.0}` (가상 시험: `virtual: true`)
- 실행: `python3 run_real.py --gripper drl ...` (처음엔 `--gripper mock`)
- 준비: bringup(mode:=real), 로봇 모드 자동, 펜던트 Tool Flange 출력 24 V
- 주의: 명령 1회당 DRL 실행(시리얼 열기 → 토크 → 위치 → 대기 1 s)이라 한 번에 약 1.5 s 걸린다. 짜기(squeeze) 시간은 이 지연이 더해진다. 필요하면 `move_wait_sec` 을 줄여 보정.
- 위치 0 열림 ~ 700 닫힘 = 우리가 쓰는 hold/squeeze 값과 같은 척도.
