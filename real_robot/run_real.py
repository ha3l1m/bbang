#!/usr/bin/env python3
"""실제 로봇(두산 E0509)으로 붕봇 동작 실행 — ROS 2 Jazzy + doosan-robot2(jazzy) 기준.

먼저 다른 터미널에서 bringup_real.sh 가 떠 있어야 한다. 실행 전에 모든 명령을 미리 보여주고
(드라이런) 위험한 좌표가 있으면 거부한다. 처음엔 반드시 --gripper mock + 저속으로.

  python3 run_real.py --test-j6            # J6 만 ±10° (연결 확인)
  python3 run_real.py --step pour_batter   # 동작 하나 (add_filling cover_filling wait_bake flip_mold pick_to_bag)
  python3 run_real.py --flavor redbean     # 붕어빵 1개 전체 (--cells 3 이면 한 판 3개)
  --poses ../ros2_ws/src/boong_robot/config/poses.clicker.example.yaml  # 클리커 배치 예시
"""
import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from plan_check import PKG, dry_run, summary, validate  # noqa: E402

from boong_robot.poses import load_poses  # noqa: E402

ROBOT_ID, ROBOT_MODEL = "dsr01", "e0509"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--step", help="pour_batter | add_filling | cover_filling | wait_bake | flip_mold | pick_to_bag")
    ap.add_argument("--flavor", default="redbean", help="앙금: redbean | cream | choco, 칸별이면 쉼표로 (예: redbean,choco)")
    ap.add_argument("--cells", type=int, default=1, help="한 판에 만들 개수 (클리커 칸 수, 처음엔 1)")
    ap.add_argument("--gripper", default="mock", choices=["mock", "drl", "ros_service", "digital_out"], help="처음엔 mock(그리퍼 안 움직임)")
    ap.add_argument("--vel-x", type=float, default=50, help="직선 속도 mm/s (기본 50 = 느리게)")
    ap.add_argument("--vel-j", type=float, default=20, help="관절 속도 deg/s")
    ap.add_argument("--test-j6", action="store_true", help="J6 ±10° 만 움직여 연결 확인")
    ap.add_argument("--table", default="120,1080,275,275", help="테이블 가장자리까지 거리(mm) back,fwd,left,right — 벗어나면 경고(실행은 막지 않음)")
    ap.add_argument("--yes", action="store_true", help="확인 질문 생략")
    ap.add_argument("--poses", default=str(PKG / "config" / "poses.yaml"))
    args = ap.parse_args()

    poses = load_poses(args.poses)
    poses = {**poses, "gripper": {**poses.get("gripper", {}), "type": args.gripper},
             "speed": {"vel_j": args.vel_j, "acc_j": args.vel_j, "vel_x": args.vel_x, "acc_x": args.vel_x}}

    from boong_robot.menu import parse_menu
    args.flavor = parse_menu(args.flavor)
    if isinstance(args.flavor, list):
        args.cells = len(args.flavor)          # 칸별 메뉴 개수 = 칸 수
    cells = tuple(range(args.cells))

    def run(bot):
        if args.step == "add_filling":
            bot.add_filling(args.flavor, cells)
        elif args.step in ("pour_batter", "cover_filling", "pick_to_bag"):
            getattr(bot, args.step)(cells)
        elif args.step:
            getattr(bot, args.step)()
        else:
            bot.make_batch(args.flavor, args.cells)

    if not args.test_j6:
        calls = dry_run(poses, run)
        print("\n".join(summary(calls)))
        from arm_model import layout_warnings, parse_table
        for w in layout_warnings(poses, parse_table(args.table), calls):
            print("⚠️ 테이블:", w)
        problems = validate(calls)
        if problems:
            print("\n⚠️ 거부: 좌표를 먼저 고치세요\n" + "\n".join(problems))
            sys.exit(2)
        print(f"\n속도 {args.vel_x}mm/s · {args.vel_j}deg/s · 그리퍼={args.gripper} · 명령 {len(calls)}개")
        if not args.yes and input("비상정지 버튼을 손에 쥐었나요? 실행하려면 yes: ").strip().lower() != "yes":
            print("취소"); return

    import rclpy
    import DR_init  # PYTHONPATH 에 .../install/dsr_common2/lib/dsr_common2/imp 필요 (setup_doosan_jazzy.sh 가 안내)

    rclpy.init()
    node = rclpy.create_node("boong_real", namespace=ROBOT_ID)
    DR_init.__dsr__id, DR_init.__dsr__model, DR_init.__dsr__node = ROBOT_ID, ROBOT_MODEL, node
    from DSR_ROBOT2 import ROBOT_MODE_AUTONOMOUS, set_robot_mode  # 노드 설정 '후에' import

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    from boong_robot.backends import DsrBackend
    from boong_robot.menu import MenuBungeoppang as Bungeoppang

    backend = DsrBackend(poses["speed"], log=print, tool=poses.get("tool"))
    try:
        if args.test_j6:
            print("J6 +10° → -10°")
            backend.rotate_j6(10); backend.rotate_j6(-10)
        else:
            bot = Bungeoppang(backend, poses, on_step=lambda s, d: print(f"[{s}] {d}"))
            run(bot)
            if args.step:
                bot.home()
        print("끝")
    except KeyboardInterrupt:
        print("중단 (비상정지로 로봇을 먼저 멈추세요)")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
