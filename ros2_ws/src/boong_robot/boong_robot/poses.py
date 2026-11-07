"""poses.yaml 읽기 + 좌표 계산 도우미 (ROS 의존성 없음)."""
import math
from pathlib import Path

import yaml

REQUIRED = ["home", "batter_bottle", "redbean_bottle", "cream_bottle", "mold_dispense", "mold_handle", "bread_pick", "bag_drop"]


def load_poses(path: str | Path) -> dict:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    missing = [k for k in REQUIRED if k not in data]
    if missing:
        raise ValueError(f"poses.yaml에 없는 좌표: {missing}")
    for name, m in data.get("materials", {}).items():
        if m["bottle"] not in data:
            raise ValueError(f"materials.{name}.bottle '{m['bottle']}' 좌표가 poses.yaml에 없음")
    return data


def above(pose: list[float], dz: float) -> list[float]:
    """같은 자세로 dz(mm)만큼 위."""
    p = list(pose)
    p[2] += dz
    return p


def tool_axis(pose: list[float]) -> list[float]:
    """툴이 향한 방향(단위벡터). 두산 ZYZ 오일러(a, b)에서 계산."""
    a, b = math.radians(pose[3]), math.radians(pose[4])
    return [math.cos(a) * math.sin(b), math.sin(a) * math.sin(b), math.cos(b)]


def approach(pose: list[float], dist: float) -> list[float]:
    """툴이 향한 방향의 반대쪽으로 dist(mm) 물러난 점 (같은 자세).
    툴이 아래를 향하면 '위', 수평이면 '옆(뒤)'이 되어 접근/이탈 경로가 자연스럽다."""
    d = tool_axis(pose)
    p = list(pose)
    for i in range(3):
        p[i] = round(p[i] - dist * d[i], 4)
    return p


def spun(pose: list[float], deg: float) -> list[float]:
    """툴 z축 기준 회전 (ZYZ의 c 각도). J6을 deg만큼 돌린 뒤의 같은 위치."""
    p = list(pose)
    p[5] += deg
    return p


def cell_pose(pose: list[float], i: int, pitch: float) -> list[float]:
    """i번째 클리커 칸의 같은 자세 좌표 (칸은 +y 방향으로 pitch mm 간격)."""
    p = list(pose)
    p[1] = round(p[1] + i * pitch, 4)
    return p
