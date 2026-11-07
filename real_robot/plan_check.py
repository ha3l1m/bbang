"""로봇을 움직이기 전에 계획된 모든 명령을 미리 뽑아 보고(드라이런) 위험한 좌표를 걸러낸다. ROS 불필요."""
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PKG = REPO / "ros2_ws" / "src" / "boong_robot"
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(REPO / "rviz_demo"))

from boong_robot.backends import MockBackend  # noqa: E402
from boong_robot.menu import MenuBungeoppang as Bungeoppang  # noqa: E402  (칸별 메뉴 지원)
from boong_robot.poses import load_poses  # noqa: E402

REACH = 900.0      # E0509 (라벨: Reach 900.0 mm)
SAFE_REACH = 850.0  # 이 이상은 끝까지 펴야 해서 거부
MIN_Z = 30.0        # 베이스 평면 기준 최소 높이(mm) — 테이블에 박지 않게. TCP 설정에 맞춰 조정


def dry_run(poses, run):
    b = MockBackend(log=lambda *_: None, speed=0)
    bot = Bungeoppang(b, {**poses, "gripper": {**poses.get("gripper", {}), "type": "mock"}})
    run(bot)
    return b.calls


def validate(calls):
    problems = []
    for i, c in enumerate(calls, 1):
        if c[0] != "movel":
            continue
        x = c[1]
        if x[2] < MIN_Z:
            problems.append(f"#{i} movel {x[:3]} : z={x[2]} < {MIN_Z}mm (테이블/바닥에 닿을 수 있음)")
        r = math.dist((0, 0, 0), x[:3])
        if r > SAFE_REACH:
            problems.append(f"#{i} movel {x[:3]} : 베이스에서 {r:.0f}mm > {SAFE_REACH:.0f}mm (팔이 거의 다 펴짐/못 닿음)")
    return problems


def summary(calls):
    lines = []
    for i, c in enumerate(calls, 1):
        if c[0] == "wait":
            continue
        lines.append(f"{i:3d}  {c[0]:9s} {c[1] if len(c) > 1 else ''}")
    return lines


if __name__ == "__main__":
    poses = load_poses(PKG / "config" / "poses.yaml")
    calls = dry_run(poses, lambda bot: bot.make_batch("redbean", min(3, int(poses.get("cells", {}).get("count", 1)))))
    print("\n".join(summary(calls)))
    probs = validate(calls)
    print("\n경고 없음" if not probs else "\n⚠️ 경고:\n" + "\n".join(probs))
