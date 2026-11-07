"""붕봇 RViz 데모용 그림 계산 (ROS 의존성 없음 → 로봇·ROS 없이 테스트 가능).

팔은 '근사 모델'이다: 어깨 높이 155mm, 위팔 409mm, 아래팔 367mm, 툴 160mm 의 2링크.
실제 E0509 URDF 가 아니라, 툴 끝이 어디로 가는지·통이 어떻게 뒤집히는지 보는 용도다.
(실제 로봇 모델로 보려면 docker 에뮬레이터 + `./robot.sh bringup` 을 쓴다.)
단위: 입력은 mm/deg, 출력 마커는 m.
"""
import math

from boong_robot import plate_frame

L1, L2, SH, TOOL = 409.0, 367.0, 155.0, 160.0
BOT_H, BOT_W = 210.0, 65.0          # 소스통(400ml) 높이/지름 (mm) — 사진으로 추정, 실측해서 고칠 것
CL, CT, TOP_Z = 90.0, 30.0, 75.0   # 클리커 한쪽: 길이(y 방향)·두께, 받침 위 윗면 높이 (mm)
MOLD_TOP = TOP_Z

BATTER = (0.98, 0.92, 0.75)
REDBEAN = (0.55, 0.29, 0.17)
CREAM = (0.96, 0.85, 0.54)
GOLD = (0.96, 0.65, 0.14)
ROBOT = (0.23, 0.40, 0.66)
STAMP = (0.78, 0.26, 0.17)
CHOCO = (0.33, 0.19, 0.11)
MAT_COLOR = {"batter": BATTER, "redbean": REDBEAN, "cream": CREAM, "choco": CHOCO}
BOTTLES = ("batter_bottle", "redbean_bottle", "cream_bottle", "choco_bottle")   # poses.yaml 에 없는 통은 건너뜀
LABEL = {"batter_bottle": "batter", "redbean_bottle": "red bean", "cream_bottle": "cream", "choco_bottle": "choco"}


def clamp(v, a, b):
    return max(a, min(b, v))


def lerp(a, b, t):
    return a + (b - a) * t


# ---------- 회전 ----------
def _mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _rz(d):
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def _ry(d):
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rot_zyz(a, b, c):
    return _mm(_mm(_rz(a), _ry(b)), _rz(c))


def col(R, j):
    return [R[0][j], R[1][j], R[2][j]]


def tool_axes(t):
    """툴 좌표계의 (x, y, z) 축 (세계 기준 단위벡터)."""
    R = rot_zyz(t[3], t[4], t[5])
    return col(R, 0), col(R, 1), col(R, 2)


def quat_z_to(v):
    """z축을 v 방향으로 돌리는 쿼터니언 (x,y,z,w)."""
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    v = [c / n for c in v]
    d = v[2]
    if d < -0.999999:
        return (1.0, 0.0, 0.0, 0.0)
    q = (-v[1], v[0], 0.0, 1.0 + d)  # cross(z, v) = (-vy, vx, 0)
    m = math.sqrt(sum(c * c for c in q))
    return tuple(c / m for c in q)


def quat_y(deg):
    h = math.radians(deg) / 2
    return (0.0, math.sin(h), 0.0, math.cos(h))


def quat_mul(a, b):
    """a ⊗ b  (x, y, z, w)"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


# ---------- 팔 ----------
def arm_points(t):
    """베이스·어깨·팔꿈치·손목·툴 끝 (mm). 툴 끝(t)은 입력 그대로."""
    _, _, z = tool_axes(t)
    W = [t[i] - z[i] * TOOL for i in range(3)]
    phi = math.atan2(W[1], W[0])
    rw, zw = math.hypot(W[0], W[1]), W[2] - SH
    Dc = max(1.0, min(math.hypot(rw, zw), L1 + L2 - 1))
    th = math.atan2(zw, rw)
    B = math.acos(clamp((L1 * L1 + Dc * Dc - L2 * L2) / (2 * L1 * Dc), -1, 1))
    er, ez = L1 * math.cos(th + B), SH + L1 * math.sin(th + B)
    E = [er * math.cos(phi), er * math.sin(phi), ez]
    reach = math.hypot(rw, zw) <= L1 + L2
    return {"base": [0, 0, 0], "shoulder": [0, 0, SH], "elbow": E, "wrist": W, "tip": list(t[:3]), "reach": reach}


def finger_offset(gp, bottle):
    if bottle:
        return lerp(62, BOT_W / 2 + 5, clamp(gp / 380, 0, 1)) if gp <= 380 else (BOT_W / 2 + 5) - 13 * clamp((gp - 380) / 300, 0, 1)
    return 8 + 54 * (1 - clamp(gp / 700, 0, 1))


# 테이블(로봇 베이스 중심 기준 가장자리까지 거리, mm) — 사진으로 추정한 값. 줄자로 재서 --table 로 바꾸세요.
TABLE_DEFAULT = {"back": 120.0, "fwd": 1080.0, "left": 275.0, "right": 275.0}


def parse_table(text):
    """'back,fwd,left,right' (mm) → dict"""
    v = [float(t) for t in text.split(",")]
    if len(v) != 4:
        raise ValueError("--table 은 back,fwd,left,right 4개 숫자 (mm)")
    return dict(zip(("back", "fwd", "left", "right"), v))


def outside_table(xy, table, margin=30.0):
    """툴 끝 (x,y) 가 테이블 가장자리에서 margin 안쪽을 벗어나면 얼마나 벗어났는지(mm), 아니면 0."""
    x, y = xy
    over = [x - (table["fwd"] - margin), -(x + table["back"] - margin) , y - (table["left"] - margin), -(y + table["right"] - margin)]
    return max(0.0, *over)


def layout_warnings(poses, table, calls):
    """드라이런으로 뽑은 명령 + 고정 물체가 테이블 밖으로 나가는지 검사 → 경고 문장 목록."""
    out, seen = [], set()
    for c in calls:
        if c[0] != "movel":
            continue
        o = outside_table(c[1][:2], table)
        if o > 0 and (c[1][0], c[1][1]) not in seen:
            seen.add((c[1][0], c[1][1]))
            out.append(f"툴 끝 ({c[1][0]:.0f}, {c[1][1]:.0f}) 이 테이블 가장자리 30mm 안쪽 선보다 {o:.0f}mm 밖")
    for name, r in [(b, BOT_W / 2) for b in BOTTLES if b in poses]:
        x, y = poses[name]["posx"][:2]
        for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r)):
            if outside_table((x + dx, y + dy), table, margin=0):
                out.append(f"{name} 통이 테이블 밖에 걸침 ({x:.0f}, {y:.0f})"); break
    cfg = poses.get("cells", {})
    mx, my = poses["mold_dispense"]["posx"][:2]
    for i in range(int(cfg.get("count", 1))):
        cy = my + i * float(cfg.get("pitch_mm", 0))
        if any(outside_table((mx + dx, cy + dy), table, 0) for dx in (-30, 30) for dy in (-CL / 2, CL / 2)):
            out.append(f"클리커 {i}번 칸이 테이블 밖에 걸침 ({mx:.0f}, {cy:.0f})")
    bx, by = poses["bag_drop"]["posx"][:2]
    if outside_table((bx, by - 50), table, 0) or outside_table((bx, by + 50), table, 0):
        out.append(f"봉투가 테이블 밖에 걸침 ({bx:.0f}, {by:.0f})")
    return out


class Vis:
    """지금 화면에 그릴 모든 상태."""

    def __init__(self, poses):
        self.p = poses
        self.table = dict(TABLE_DEFAULT)
        self.warn = []
        self.mat_of_bottle = {m["bottle"]: name for name, m in poses.get("materials", {}).items()}
        self.reset()

    def reset(self):
        self.tool = [370.0, 0.0, 440.0, 0.0, 180.0, 0.0]
        self.gp = 0.0            # 그리퍼 위치 0 열림 ~ 700 닫힘
        self.held = None         # 'batter_bottle' | 'mold_handle' | 'bread_pick' | None
        self.bread = "empty"     # empty batter filling covered baked flipped bagged
        self.bread_color = BATTER
        self.flip = 0.0          # 커튼봉(뚜껑 축) 회전각: 0 열림 ~ 180 닫힘
        self.turn = 0.0          # 철판: 닫힌 묶음을 통째로 뒤집은 각도 0 ~ 180 (로봇 쪽으로 넘어감)
        self.picked = set()      # 봉투로 옮긴 칸 번호
        self.held_cell = None    # 붕어빵을 집었을 때 몇 번 칸인지
        self.n_active = int(self.p.get('cells', {}).get('count', 1))   # 이번에 쓰는 칸 수
        self.pour = 0.0          # 짜는 중 0..1 (0이면 안 짜는 중)
        self.pour_mat = None
        self.cell_colors = {}    # 칸별 앙금 색 {칸: rgb} (칸별 메뉴일 때)
        self.label = "ready"

    def station_at(self, tol=3.0):
        best, bd = None, tol
        for name in [b for b in BOTTLES if b in self.p] + ["mold_handle"]:
            p = self.p[name]["posx"]
            spots = [p]
            if name == "mold_handle" and plate_frame.is_plate(self.p.get("frame")):   # 철판: 닫힌 뚜껑 판 가장자리에서도 잡는다
                spots.append(plate_frame.grip_pose(p, self.p["frame"], 180.0))
                for d in (0.0, 180.0):                                              # 닫힌 묶음(뒤집기용) 잡는 곳
                    q = plate_frame.stack_pose(p, self.p["frame"], d)
                    if math.dist(q[:3], self.tool[:3]) <= bd:
                        best, bd = "mold_stack", math.dist(q[:3], self.tool[:3])
            elif name == "mold_handle" and self.p.get("frame"):      # 사다리 틀: 닫힌 자리(경첩 건너편)에서도 잡는다
                spots.append([p[0] + 2 * self.p["frame"]["span_mm"]] + list(p[1:]))
            for q in spots:
                d = math.dist(q[:3], self.tool[:3])
                if d <= bd:
                    best, bd = name, d
        pitch = float(self.p.get("cells", {}).get("pitch_mm", 0))
        for i in range(int(self.p.get("cells", {}).get("count", 1))):
            p = self.p["bread_pick"]["posx"]
            d = math.dist([p[0], p[1] + i * pitch, p[2]], self.tool[:3])
            if d <= bd:
                best, bd, self.held_cell = "bread_pick", d, i
        return best


def _m(mid, typ, pos, scale, color, quat=(0, 0, 0, 1), **kw):
    d = {"id": mid, "type": typ, "pos": [v / 1000.0 for v in pos], "quat": quat,
         "scale": [v / 1000.0 for v in scale], "color": (*color, kw.pop("alpha", 1.0))}
    d.update(kw)
    return d


def _plate_markers(v, P, fr, out, n, pitch, pw, cl, mx, my0):
    """철판 뚜껑 틀: 받침 박스 + 아래 판(틀 아래 반쪽·음각) + 뚜껑 판(위 반쪽) — 뚜껑은 경첩 둘레로 v.flip 만큼 넘어간다."""
    grip = P["mold_handle"]["posx"]
    px, pz = plate_frame.pivot(grip, fr)
    zp = grip[2]                                   # 판 면 높이 (= 받침 박스 높이)
    half = float(fr["closed_mm"]) / 2              # 틀 반쪽 두께
    depth, length = (float(x) for x in fr["plate_mm"])
    cells_y = [my0 + i * pitch for i in range(n)]
    yc = (cells_y[0] + cells_y[-1]) / 2
    q = quat_y(v.flip)
    mesh = (0.75, 0.77, 0.8)
    # 받침 박스: 판보다 작게 (+y 쪽 가장자리 35mm 비워 손가락 틈)
    bx0, bx1, by0, by1 = px - depth + 20, px + depth - 20, yc - length / 2 + 10, yc + length / 2 - 35
    out.append(_m(132, "cube", [(bx0 + bx1) / 2, (by0 + by1) / 2, zp / 2 - 1], [bx1 - bx0, by1 - by0, zp - 2], (0.62, 0.5, 0.38), alpha=0.85))
    # 아래 판 (고정, +x 쪽)
    out.append(_m(130, "cube", [px + depth / 2, yc, zp - 1.5], [depth, length, 3], mesh, alpha=0.55))
    # 뚜껑 판 (열림: -x 쪽에 펼침 → 경첩 둘레로 회전)
    lx, lz = plate_frame.rotate_about_pivot((px - depth / 2, zp - 1.5), (px, pz), v.flip)
    out.append(_m(131, "cube", [lx, yc, lz], [depth, length, 3], mesh, q, alpha=0.55))
    for i, cy in enumerate(cells_y):
        col_b = (0.55, 0.72, 0.9) if i < v.n_active else (0.7, 0.7, 0.7)
        # 아래 반쪽 (경첩 바로 +x 쪽) + 붕어빵 음각
        out.append(_m(100 + i, "cube", [mx, cy, zp + half / 2], [pw, cl, half], col_b, alpha=0.95))
        eng, zt = (0.22, 0.18, 0.15), zp + half + 0.5
        out.append(_m(150 + i, "sphere", [mx, cy - 0.04 * cl, zt], [0.56 * pw, 0.66 * cl, 3], eng))
        out.append(_m(160 + i, "cube", [mx, cy + 0.40 * cl, zt], [0.20 * pw, 0.20 * pw, 3], eng, (0.0, 0.0, 0.38268, 0.92388)))
        out.append(_m(170 + i, "sphere", [mx + 0.14 * pw, cy - 0.28 * cl, zt + 2], [5, 5, 3], (0.95, 0.95, 0.95)))
        # 위 반쪽 (뚜껑 판에 붙어 같이 회전). 열림: 경첩 -x 쪽 대칭 위치
        ux, uz = plate_frame.rotate_about_pivot((2 * px - mx, zp + half / 2), (px, pz), v.flip)
        out.append(_m(120 + i, "cube", [ux, cy, uz], [pw, cl, half], col_b, q, alpha=0.9))
    out.append(_m(43, "text", [px, cells_y[0], 5], [0, 0, 30], (1, 1, 1), text="plate mold x%d" % n))
    return cells_y



def build_markers(v: Vis):
    """현재 상태 → 마커 목록(dict). id 는 항상 고정이라 RViz 에서 부드럽게 갱신된다."""
    P, out = v.p, []
    A = arm_points(v.tool)
    if getattr(v, 'show_arm', True):   # run_sim.py 는 실제 E0509/그리퍼 모델을 쓰므로 근사 팔은 끈다
        # --- 팔 ---
        out.append(_m(1, "line_strip", [0, 0, 0], [30, 0, 0], ROBOT, points=[A["base"], A["shoulder"], A["elbow"], A["wrist"]], alpha=0.9))
        out[-1]["points"] = [[c / 1000.0 for c in p] for p in out[-1]["points"]]
        out.append(_m(2, "line_strip", [0, 0, 0], [14, 0, 0], ROBOT, points=[[c / 1000.0 for c in A["wrist"]], [c / 1000.0 for c in A["tip"]]]))
        out.append(_m(3, "sphere_list", [0, 0, 0], [40, 40, 40], (1, 1, 1), points=[[c / 1000.0 for c in A[k]] for k in ("shoulder", "elbow", "wrist")]))
        out.append(_m(4, "cube", [0, 0, SH / 2], [120, 120, SH], ROBOT, alpha=0.5))
        if not A["reach"]:
            out.append(_m(5, "text", [A["tip"][0], A["tip"][1], A["tip"][2] + 120], [0, 0, 50], STAMP, text="out of reach!"))
        # --- 손가락 ---
        held_bottle = v.held in BOTTLES
        off = finger_offset(v.gp, held_bottle)
        _, ya, _ = tool_axes(v.tool)
        tips = [[v.tool[i] + s * off * ya[i] for i in range(3)] for s in (1, -1)]
        out.append(_m(6, "sphere_list", [0, 0, 0], [22, 22, 22], STAMP if v.gp > 0 else (1, 1, 1), points=[[c / 1000.0 for c in p] for p in tips]))
    # --- 소스통 3개 ---
    for i, name in enumerate(BOTTLES):
        if name not in P:
            continue
        mat = v.mat_of_bottle.get(name, "batter")
        if v.held == name:
            xa, _, _ = tool_axes(v.tool)
            axis = [-c for c in xa]            # 노즐 방향: c=0 이면 위, 180 이면 아래
            ctr = v.tool[:3]
            sf = clamp((v.gp - 380) / 300, 0, 1)
        else:
            axis, sf = [0, 0, 1], 0.0
            ctr = list(P[name]["posx"][:3])
        w = BOT_W * (1 - 0.37 * sf)
        q = quat_z_to(axis)
        # 통 전체 길이 BOT_H = 몸통 + 노즐 캡(24mm). 가운데(ctr)가 툴 끝이 잡는 자리.
        body = [ctr[k] - axis[k] * 12 for k in range(3)]
        out.append(_m(10 + i, "cylinder", body, [w, w, BOT_H - 24], MAT_COLOR[mat], q, alpha=0.95))
        cap = [ctr[k] + axis[k] * (BOT_H / 2 - 12) for k in range(3)]
        out.append(_m(20 + i, "cylinder", cap, [22, 22, 24], (0.2, 0.15, 0.1), q))
        if v.held != name:
            out.append(_m(30 + i, "text", [ctr[0], ctr[1], ctr[2] + BOT_H / 2 + 60], [0, 0, 36], (1, 1, 1), text=LABEL[name]))
    # --- 클리커 칸들 + 커튼봉 ---
    cfg = P.get("cells", {})
    n, pitch = int(cfg.get("count", 1)), float(cfg.get("pitch_mm", 0))
    pw, cl = (float(x) for x in cfg.get("plate_mm", [50.0, CL]))   # 틀 판 하나: 가로(x) × 세로(y) mm
    mx, my0 = P["mold_dispense"]["posx"][:2]
    ax_x, hy, ax_z = P["mold_handle"]["posx"][0], P["mold_handle"]["posx"][1], P["mold_handle"]["posx"][2]
    fr = P.get("frame")
    if plate_frame.is_plate(fr):
        cells_y = _plate_markers(v, P, fr, out, n, pitch, pw, cl, mx, my0)
        mcx, cw = mx, pw
    else:
        if fr:      # 사다리 뚜껑 틀: 경첩 봉(가운데 고리)은 ax_x + span, 뚜껑 폭 = span
            cw = float(fr["span_mm"]); ax_x += cw; mcx = ax_x + cw / 2
        else:       # 예전 방식: 봉 하나가 경첩
            cw = max(30.0, 2 * (mx - ax_x)); mcx = mx
        closed = (v.flip % 360) > 90 and (v.flip % 360) < 270
        ry = math.radians(v.flip)
        cells_y = [my0 + i * pitch for i in range(n)]
        for i, cy in enumerate(cells_y):
            active = i < v.n_active
            col_b = (0.55, 0.72, 0.9) if active else (0.7, 0.7, 0.7)
            out.append(_m(100 + i, "cube", [mcx, cy, TOP_Z - CT / 2], [cw, cl, CT], col_b, alpha=0.95))
            # 붕어빵 음각(오목한 자리): 몸통(타원) + 꼬리(마름모) + 눈. 머리 -y, 꼬리 +y
            eng, zt = (0.22, 0.18, 0.15), TOP_Z + 0.5
            out.append(_m(150 + i, "sphere", [mcx, cy - 0.04 * cl, zt], [0.56 * cw, 0.66 * cl, 3], eng))
            out.append(_m(160 + i, "cube", [mcx, cy + 0.40 * cl, zt], [0.20 * cw, 0.20 * cw, 3], eng, (0.0, 0.0, 0.38268, 0.92388)))
            out.append(_m(170 + i, "sphere", [mcx + 0.14 * cw, cy - 0.28 * cl, zt + 2], [5, 5, 3], (0.95, 0.95, 0.95)))
            # 뚜껑: 봉 축(x=ax_x, z=ax_z) 기준으로 회전. flip=0 이면 로봇 쪽(-x)으로 펼쳐져 있고 180 이면 칸 위에 덮임
            lx = ax_x + math.cos(ry) * (-cw / 2) + math.sin(ry) * (-8)
            lz = ax_z - math.sin(ry) * (-cw / 2) + math.cos(ry) * (-8)
            out.append(_m(120 + i, "cube", [lx, cy, lz], [cw, cl, 12], col_b, quat_y(v.flip), alpha=0.9))
        y0, y1 = min(cells_y) - cl / 2 - 25, max(hy + 25, max(cells_y) + cl / 2 + 25)
        out.append(_m(130, "cylinder", [ax_x, (y0 + y1) / 2, ax_z], [18, 18, y1 - y0], (0.95, 0.95, 0.95), quat_z_to([0, 1, 0])))
        if fr:      # 움직이는 봉 + 고리 3곳(양 끝) — 뚜껑과 같이 반원을 그린다
            mxr, mzr = ax_x - cw * math.cos(ry), ax_z + cw * math.sin(ry)
            out.append(_m(133, "cylinder", [mxr, (y0 + y1) / 2, mzr], [18, 18, y1 - y0], (0.95, 0.75, 0.3), quat_z_to([0, 1, 0])))
            for a, hx in enumerate((ax_x - cw, ax_x, ax_x + cw)):
                for b2, yy in enumerate((y0 + 8, y1 - 8)):
                    out.append(_m(134 + a * 2 + b2, "cube", [hx, yy, ax_z / 2 - 8], [26, 26, ax_z - 16], (0.4, 0.4, 0.4)))   # 고리(받침)
        else:
            for k, yy in enumerate((y0 + 20, y1 - 60)):
                out.append(_m(131 + k, "cube", [ax_x, yy, ax_z / 2 - 8], [30, 30, ax_z - 16], (0.4, 0.4, 0.4)))   # 받침(축받이)
        out.append(_m(41, "cube", [mcx, (cells_y[0] + cells_y[-1]) / 2, (TOP_Z - CT) / 2], [cw + 80, (n - 1) * pitch + cl + 80, TOP_Z - CT], (0.43, 0.35, 0.29), alpha=0.5))
        out.append(_m(43, "text", [mcx, cells_y[0], 5], [0, 0, 30], (1, 1, 1), text="clicker x%d" % n))
    # --- 붕어빵 (칸마다) ---
    for i, cy in enumerate(cells_y):
        if i >= v.n_active or v.bread == "empty":
            continue
        if v.held == "bread_pick" and v.held_cell == i:
            bp = list(v.tool[:3])
        elif i in v.picked:
            bx, by = P["bag_drop"]["posx"][:2]
            bp = [bx, by - 30 + 30 * (len([j for j in v.picked if j < i])), 60]
        else:
            bp = [mx, cy, TOP_Z + 8]
        color = {"batter": BATTER, "filling": v.cell_colors.get(i, v.bread_color), "covered": BATTER, "baked": GOLD, "flipped": GOLD, "bagged": GOLD}[v.bread]
        out.append(_m(140 + i, "sphere", bp, [0.62 * cw, 0.7 * cl, 20], color))
    # --- 짜는 중: 줄기 + 칸 위에 퍼지는 덩어리 ---
    if v.pour > 0 and v.held in BOTTLES:
        xa, _, _ = tool_axes(v.tool)
        nz = [v.tool[k] + (-xa[k]) * (BOT_H / 2) for k in range(3)]  # 노즐 끝
        mat = v.pour_mat or "batter"
        h = max(4.0, nz[2] - TOP_Z)
        w = 4 + 6 * v.pour
        cy = min(cells_y, key=lambda c: abs(c - nz[1]))
        out.append(_m(60, "cylinder", [nz[0], nz[1], nz[2] - h / 2], [w, w, h], MAT_COLOR[mat]))
        out.append(_m(61, "sphere", [mx, cy, TOP_Z + 4], [0.1 * cw + 0.45 * cw * v.pour, 0.15 * cl + 0.5 * cl * v.pour, 8], MAT_COLOR[mat]))
    # --- 봉투 ---
    bx, by = P["bag_drop"]["posx"][:2]
    out.append(_m(70, "cube", [bx, by, 80], [140, 100, 160], (0.91, 0.81, 0.64), alpha=0.7))
    out.append(_m(71, "text", [bx, by, 200], [0, 0, 36], (1, 1, 1), text="bag"))
    out.append(_m(72, "text", [mx, cells_y[len(cells_y) // 2], 460], [0, 0, 50], (1, 1, 0.6), text=v.label))
    # --- 테이블 (윗면 z=0) ---
    t = v.table
    cx, cy = (t["fwd"] - t["back"]) / 2, (t["left"] - t["right"]) / 2
    out.append(_m(80, "cube", [cx, cy, -10], [t["fwd"] + t["back"], t["left"] + t["right"], 20], (0.85, 0.85, 0.82), alpha=0.45))
    if v.warn:
        out.append(_m(81, "text", [cx, cy, 300], [0, 0, 45], STAMP, text="LAYOUT WARNING: %d items off table" % len(v.warn)))
    if fr and plate_frame.is_plate(fr) and v.turn:
        _turn_stack(out, v, P, fr)
    return out


def _turn_stack(out, v, P, fr):
    """닫힌 철판 묶음(두 판·틀 4쪽·음각·틀 안의 붕어빵)을 경첩 둘레로 '위로' v.turn 만큼 넘긴다 (로봇 쪽 -x 로)."""
    px, pz = plate_frame.pivot(P["mold_handle"]["posx"], fr)
    group = {130, 131} | set(range(100, 110)) | set(range(120, 130)) | set(range(150, 180))
    for i in range(10):                                        # 틀 안에 있는 붕어빵만
        if not (v.held == "bread_pick" and v.held_cell == i) and i not in v.picked:
            group.add(140 + i)
    qt = quat_y(-v.turn)
    for d in out:
        if d["id"] in group:
            x, z = plate_frame.rotate_about_pivot((d["pos"][0] * 1000, d["pos"][2] * 1000), (px, pz), -v.turn)
            d["pos"] = [x / 1000, d["pos"][1], z / 1000]
            d["quat"] = quat_mul(qt, d["quat"])
