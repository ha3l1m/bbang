#!/usr/bin/env python3
"""붕봇 RViz 데모: 두산 패키지·에뮬레이터·Docker 없이 '동작 순서'를 RViz 에서 눈으로 확인.

  motions.py 가 내보내는 명령(movel / rotate_j6 / 그리퍼)을 그대로 받아서
  근사 팔 + 소스통 + 틀 + 봉투를 /boong/viz (MarkerArray) 로 그린다.

  python3 viz_demo.py --flavor redbean --count 1 --speed 3
  python3 viz_demo.py --step pour_batter          # 동작 하나만
"""
import argparse
import math
import os
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = Path(os.environ.get("BOONG_REPO", HERE.parent))
PKG = REPO / "ros2_ws" / "src" / "boong_robot"
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(HERE))

import rclpy  # noqa: E402
from geometry_msgs.msg import Point  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

from arm_model import BATTER, MAT_COLOR, Vis, build_markers, clamp, layout_warnings, lerp, parse_table  # noqa: E402
from boong_robot.gripper import Gripper  # noqa: E402
from boong_robot.menu import MenuBungeoppang as Bungeoppang, parse_menu  # noqa: E402  (칸별 메뉴)
from boong_robot.poses import load_poses  # noqa: E402

TYPES = {"cube": Marker.CUBE, "sphere": Marker.SPHERE, "cylinder": Marker.CYLINDER, "line_strip": Marker.LINE_STRIP,
         "line_list": Marker.LINE_LIST, "sphere_list": Marker.SPHERE_LIST, "text": Marker.TEXT_VIEW_FACING}
FPS = 30
# --step 으로 한 동작만 볼 때 그 동작 직전의 붕어빵 상태
PRESET = {"add_filling": "batter", "cover_filling": "filling", "wait_bake": "covered", "flip_mold": "baked", "pick_to_bag": "flipped"}



class Publisher:
    def __init__(self, node, vis, frame="base_link"):
        self.node, self.vis, self.frame = node, vis, frame
        self.pub = node.create_publisher(MarkerArray, "/boong/viz", 10)
        self.prev, self.lock = set(), threading.Lock()

    def publish(self):
        with self.lock:
            arr, now, ids = MarkerArray(), self.node.get_clock().now().to_msg(), set()
            for d in build_markers(self.vis):
                m = Marker()
                m.header.frame_id, m.header.stamp = self.frame, now
                m.ns, m.id, m.type, m.action = "boong", d["id"], TYPES[d["type"]], Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, d["pos"])
                m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, d["quat"])
                m.scale.x, m.scale.y, m.scale.z = map(float, d["scale"])
                m.color.r, m.color.g, m.color.b, m.color.a = map(float, d["color"])
                for p in d.get("points", []):
                    m.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
                m.text = d.get("text", "")
                arr.markers.append(m)
                ids.add(d["id"])
            for gone in self.prev - ids:  # 사라진 마커(놓은 통의 노즐 줄기 등)는 지운다
                m = Marker()
                m.header.frame_id, m.ns, m.id, m.action = self.frame, "boong", gone, Marker.DELETE
                arr.markers.append(m)
            self.prev = ids
            self.pub.publish(arr)


def ease(u):
    return u * u * (3 - 2 * u)


class RvizBackend:
    """MockBackend 와 같은 인터페이스 (home / move_line / rotate_j6 / set_do / wait). 부드럽게 움직이며 그린다."""

    def __init__(self, vis, pub, poses, speed=3.0):
        self.v, self.pub, self.speed = vis, pub, speed
        sp = poses.get("speed", {})
        self.vx, self.vj = sp.get("vel_x", 100), sp.get("vel_j", 30)
        self.home_tcp = [370.0, 0.0, 440.0, 0.0, 180.0, 0.0]
        self.calls = []

    def _animate(self, dur, step):
        n = max(1, int(dur * FPS))
        for i in range(1, n + 1):
            step(ease(i / n))
            self.pub.publish()
            time.sleep(1.0 / FPS)

    def home(self, posj):
        self.calls.append(("home",))
        a, b = list(self.v.tool), list(self.home_tcp)
        self._animate(max(0.6, 1.5 / self.speed), lambda u: self.v.tool.__setitem__(slice(0, 6), [lerp(a[k], b[k], u) for k in range(6)]))

    def move_line(self, posx):
        self.calls.append(("movel", tuple(round(x, 1) for x in posx)))
        a, b = list(self.v.tool), list(posx)
        dist = math.dist(a[:3], b[:3])
        ang = max(abs(a[k] - b[k]) for k in (3, 4, 5))
        dur = max(0.3, dist / (self.vx * self.speed), ang / (self.vj * 2 * self.speed))
        fr = self.v.p.get("frame")
        f0 = self.v.flip
        # 사다리 틀: 봉 끝을 잡고 있으면 툴 롤 = 뚜껑 각도 (roll_sign 부호 반영)
        f1 = max(0.0, min(180.0, (b[5] - self.v.p["mold_handle"]["posx"][5]) / float(fr.get("roll_sign", -1)))) if fr and self.v.held == "mold_handle" else None

        # 철판: 닫힌 묶음을 잡고 있으면 툴 롤 = 뒤집은 각도 (180 - (c - c0)/roll_sign)
        t0, t1 = self.v.turn, None
        if fr and self.v.held == "mold_stack":
            t1 = max(0.0, min(180.0, 180.0 - (b[5] - self.v.p["mold_handle"]["posx"][5]) / float(fr.get("roll_sign", -1))))

        def step(u):
            self.v.tool[0:6] = [lerp(a[k], b[k], u) for k in range(6)]
            if f1 is not None:
                self.v.flip = lerp(f0, f1, u)
            if t1 is not None:
                self.v.turn = lerp(t0, t1, u)
        self._animate(dur, step)

    def rotate_j6(self, deg):
        self.calls.append(("rotate_j6", deg))
        c0, f0 = self.v.tool[5], self.v.flip
        holding_mold = self.v.held == "mold_handle"

        def step(u):
            self.v.tool[5] = c0 + deg * u
            if holding_mold:
                self.v.flip = f0 + deg * u
        self._animate(max(0.5, abs(deg) / (self.vj * self.speed)), step)

    def set_do(self, pin, value):
        self.calls.append(("set_do", pin, value))

    def wait(self, sec, tick=None):
        self.calls.append(("wait", sec))
        dur = max(0.15, sec / self.speed)
        self._animate(dur, lambda u: tick(u) if tick else None)


class DemoGripper(Gripper):
    """여닫을 때 손가락 간격·잡은 물건·짜는 모습을 갱신한다."""

    def __init__(self, backend, cfg, vis, poses):
        super().__init__(backend, {**cfg, "type": "mock"})
        self.v = vis
        self.mat_of = {m["bottle"]: n for n, m in poses.get("materials", {}).items()}

    def _anim_gp(self, target, dur=0.5):
        g0 = self.v.gp
        self.b._animate(max(0.15, dur / self.b.speed), lambda u: setattr(self.v, "gp", lerp(g0, target, u)))

    def grip(self):
        self._anim_gp(700)
        self.v.held = self.v.station_at()

    def hold(self, pos):
        self._anim_gp(pos)
        self.v.held = self.v.station_at()

    def squeeze(self, pos, sec, back_to):
        self.v.pour_mat = self.mat_of.get(self.v.held, "batter")
        g0 = self.v.gp
        self.b._animate(0.2, lambda u: setattr(self.v, "gp", lerp(g0, pos, u)))

        def tick(u):
            self.v.pour = clamp(u, 0.02, 1.0)
        self.b.wait(sec, tick=tick)
        self.v.pour = 0.0
        self._anim_gp(back_to)

    def release(self):
        held = self.v.held
        self._anim_gp(0)
        if held == "bread_pick":
            self.v.picked.add(self.v.held_cell)   # 봉투 위에서 놓음 → 그 칸 붕어빵은 봉투 속으로
        self.v.held = None


class DemoScene:
    def __init__(self, vis, poses):
        self.v, self.p = vis, poses

    def set_state(self, state, flavor=None):
        if state == "empty":  # 새 붕어빵 시작: 틀을 원래대로
            self.v.flip, self.v.bread, self.v.held, self.v.pour = 0.0, "empty", None, 0.0
            self.v.cell_colors = {}
            self.v.turn = 0.0
            self.v.picked = set()
        else:
            self.v.bread = state
        if state == "filling" and isinstance(flavor, dict):     # 칸별 메뉴 {칸: 맛}
            self.v.cell_colors = {c: MAT_COLOR.get(f, BATTER) for c, f in flavor.items()}
        elif state == "filling" and flavor:
            self.v.bread_color = MAT_COLOR.get(flavor, BATTER)
            self.v.cell_colors = {}
        self.v.label = f"state: {state}"
        print(f"  [scene] {state}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--flavor", default="redbean", type=parse_menu, help="앙금: redbean | cream | choco, 칸별이면 쉼표로 (예: redbean,choco)")
    ap.add_argument("--count", type=int, default=1, help="붕어빵 총 개수")
    ap.add_argument("--cells", type=int, default=0, help="한 판에 쓰는 클리커 칸 수 (0 = poses.yaml cells.count 와 --count 중 작은 값)")
    ap.add_argument("--speed", type=float, default=3.0, help="재생 배속 (1 = poses.yaml 속도 그대로, 기본 3배)")
    ap.add_argument("--step", help="하나만 실행: pour_batter | add_filling | cover_filling | wait_bake | flip_mold | pick_to_bag")
    ap.add_argument("--loop", action="store_true", help="계속 반복")
    ap.add_argument("--table", default="120,1080,275,275", help="테이블 가장자리까지 거리(mm): back,fwd,left,right (베이스 중심 기준, 줄자로 잰 값)")
    ap.add_argument("--exit", action="store_true", help="끝나면 바로 종료 (기본은 마지막 화면을 계속 보여줌)")
    ap.add_argument("--poses", default=str(PKG / "config" / "poses.yaml"))
    ap.add_argument("--startup-wait", type=float, default=3.0, help="RViz 가 뜰 때까지 대기(초)")
    args = ap.parse_args()

    poses = load_poses(args.poses)
    poses = {**poses, "gripper": {**poses.get("gripper", {}), "type": "mock"}}  # 데모는 그리퍼 서비스를 쓰지 않는다
    rclpy.init()
    node = rclpy.create_node("boong_viz_demo")
    vis = Vis(poses)
    max_cells = int(poses.get("cells", {}).get("count", 1))
    if isinstance(args.flavor, list):          # 칸별 메뉴 → 개수 = 메뉴 수
        args.count = len(args.flavor)
    per = args.cells or min(args.count, max_cells)
    per = max(1, min(per, max_cells))
    vis.n_active = per
    vis.table = parse_table(args.table)
    from boong_robot.backends import MockBackend  # 드라이런으로 테이블 밖으로 나가는 곳 찾기
    dry = MockBackend(log=lambda *_: None, speed=0)
    Bungeoppang(dry, poses).make_batch(args.flavor[:per] if isinstance(args.flavor, list) else args.flavor, per)
    vis.warn = layout_warnings(poses, vis.table, dry.calls)
    for w in vis.warn:
        print("⚠️ 배치:", w)
    pub = Publisher(node, vis)
    backend = RvizBackend(vis, pub, poses, args.speed)
    bot = Bungeoppang(backend, poses, on_step=lambda s, d: (print(f"[{s}] {d}"), setattr(vis, "label", f"{s}: {d}")),
                      scene=DemoScene(vis, poses))
    bot.gripper = DemoGripper(backend, poses.get("gripper", {}), vis, poses)

    print(f"RViz 에서 /boong/viz (MarkerArray) 를 보세요. 좌표 파일: {args.poses}")
    t0 = time.time()
    while time.time() - t0 < args.startup_wait:
        pub.publish(); time.sleep(0.2)
    try:
        while True:
            if args.step:
                vis.bread = PRESET.get(args.step, "empty")
                vis.flip = 0.0
                cells = tuple(range(per))
                if args.step == "home":
                    bot.home()
                elif args.step == "add_filling":
                    bot.add_filling(args.flavor, cells)
                elif args.step in ("pour_batter", "cover_filling", "pick_to_bag"):
                    getattr(bot, args.step)(cells)
                else:
                    getattr(bot, args.step)()
                bot.home()
            else:
                done = 0
                while done < args.count:
                    k = min(per, args.count - done)
                    vis.n_active = k
                    print(f"=== 붕어빵 {done + 1}~{done + k}/{args.count} ({args.flavor}) ===")
                    bot.make_batch(args.flavor[done:done + k] if isinstance(args.flavor, list) else args.flavor, k)
                    done += k
            print("끝. (Ctrl+C 로 종료)" if not args.loop else "다시 시작")
            if not args.loop and args.exit:
                break
            if not args.loop:
                while True:
                    pub.publish(); time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
