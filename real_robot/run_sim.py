#!/usr/bin/env python3
"""시뮬레이션: 실제 E0509 모델(두산 에뮬레이터) + RH-P12-RN-A 그리퍼 모델 + 환경(테이블/클리커 틀/소스통/붕어빵/봉투).

  1) 로봇 팔  : DsrBackend 가 에뮬레이터에 movel/movej 를 보낸다 → RViz 의 RobotModel 이 움직임 (run_real.py 와 같은 명령)
  2) 그리퍼   : <ns>/gripper/position_cmd (Int32, 0 열림 ~ 700 닫힘) → rhp12_gripper 노드 → 실제 그리퍼 모델 관절
  3) 환경     : 같은 모션 호출로 rviz_demo 의 상태(Vis)를 갱신해 /boong/viz 마커로 그린다 (근사 팔은 끔)

먼저 `ros2 launch rhp12_gripper dsr_rviz_gripper.launch.py mode:=virtual gui:=false` 와
`rviz2 -d rviz_demo/boong_real.rviz` 가 떠 있어야 한다 (sim_up.sh 참고).
환경 마커와 팔은 따로 계산해 동시에 재생한다 → 팔 속도와 약간 어긋날 수 있다(환경은 명령 좌표 기준).
"""
import argparse
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "rviz_demo"))
from plan_check import PKG, dry_run, summary, validate  # noqa: E402

from boong_robot.poses import load_poses  # noqa: E402

ROBOT_ID, ROBOT_MODEL = "dsr01", "e0509"


class FingertipBackend:
    """에뮬레이터는 TCP 생성(config_create_tcp)을 받아주지 않는다 → poses.yaml 좌표(손가락 끝 기준)를
    '플랜지가 갈 좌표'로 바꿔서 보낸다: flange = tcp - tcp_z * (툴 z축)."""

    def __init__(self, dsr, tcp_z):
        self.dsr, self.tcp_z = dsr, tcp_z

    def to_flange(self, posx):
        from arm_model import tool_axes
        _, _, z = tool_axes(posx)
        return [posx[i] - self.tcp_z * z[i] for i in range(3)] + list(posx[3:6])

    def to_tcp(self, flange):
        from arm_model import tool_axes
        _, _, z = tool_axes(flange)
        return [flange[i] + self.tcp_z * z[i] for i in range(3)] + list(flange[3:6])

    def move_line(self, posx):
        self.dsr.move_line(self.to_flange(posx))

    def __getattr__(self, name):
        return getattr(self.dsr, name)


class TfFollower:
    """실제(에뮬레이터) 로봇의 link_6 TF 를 읽어 환경 상태(vis.tool)를 갱신한다 → 잡은 소스통·틀·붕어빵이 그리퍼를 그대로 따라간다.
    TCP(손가락 끝) = 플랜지 + tcp_z * (툴 z축).  각도는 두산 posx 와 같은 ZYZ 오일러각(deg), 롤(c)은 연속값으로 펴서 쓴다."""

    def __init__(self, vis, pub, poses, tcp_z, node, rclpy):
        import math
        import tf2_ros
        self.math, self.vis, self.pub, self.tcp_z = math, vis, pub, tcp_z
        self.buf = tf2_ros.Buffer()
        self.lst = tf2_ros.TransformListener(self.buf, node)
        self.stop = False
        self.prev_c = None
        fr = poses.get("frame") or {}
        self.sign = float(fr.get("roll_sign", -1))
        self.handle_roll = float(poses["mold_handle"]["posx"][5])
        self.ok = False

    def pose(self):
        from rclpy.time import Time
        t = self.buf.lookup_transform("base_link", "link_6", Time())
        q, p = t.transform.rotation, t.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
             [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
             [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        m = self.math
        b = m.degrees(m.atan2(m.hypot(R[0][2], R[1][2]), R[2][2]))
        a = m.degrees(m.atan2(R[1][2], R[0][2]))
        c = m.degrees(m.atan2(R[2][1], -R[2][0]))
        if self.prev_c is not None:            # 롤 연속화 (±180 점프 제거)
            while c - self.prev_c > 180: c -= 360
            while c - self.prev_c < -180: c += 360
        self.prev_c = c
        zax = [R[0][2], R[1][2], R[2][2]]
        pos = [p.x * 1000 + self.tcp_z * zax[0], p.y * 1000 + self.tcp_z * zax[1], p.z * 1000 + self.tcp_z * zax[2]]
        return pos + [a, b, c]

    def step(self):
        v = self.vis
        v.tool[0:6] = self.pose()
        if v.held == "mold_handle":      # 뚜껑: 판(봉) 끝을 잡고 있으면 뚜껑 각도 = 실제 툴 롤
            v.flip = max(0.0, min(180.0, (v.tool[5] - self.handle_roll) / self.sign))
        elif v.held == "mold_stack":     # 철판 묶음 뒤집기: 뒤집은 각도 = 180 - (롤 - 처음 롤)/부호  (360° 주기라 -90~270 으로 접음)
            t = 180.0 - (v.tool[5] - self.handle_roll) / self.sign
            t = ((t + 90.0) % 360.0) - 90.0
            v.turn = max(0.0, min(180.0, t))
        self.ok = True

    def run(self):
        while not self.stop:
            try:
                self.step()
                self.pub.publish()
            except Exception:
                pass
            time.sleep(1 / 25)


class StateOnlyViz:
    """RvizBackend 에서 '팔 움직임'은 빼고(실제 로봇 TF 가 대신함) 기다림/짜기 애니메이션만 쓴다."""

    def __new__(cls, *a, **k):
        from viz_demo import RvizBackend

        class _V(RvizBackend):
            def home(self, posj): self.calls.append(("home",))
            def move_line(self, posx): self.calls.append(("movel",))
            def rotate_j6(self, deg): self.calls.append(("rotate_j6", deg))
        return _V(*a, **k)


class BothBackend:
    """DsrBackend(실제 팔) 와 RvizBackend(환경 마커) 에 같은 호출을 동시에 보낸다."""

    def __init__(self, dsr, viz):
        self.dsr, self.viz = dsr, viz
        self.debug = True
        self.speed, self._animate = viz.speed, viz._animate    # DemoGripper 가 사용

    def _both(self, name, *a):
        t = threading.Thread(target=getattr(self.viz, name), args=a, daemon=True)
        t.start()
        getattr(self.dsr, name)(*a)
        t.join()

    def home(self, posj):
        self._both("home", posj)

    def move_line(self, posx):
        self._both("move_line", posx)
        f = getattr(self, "follower", None)
        if f is not None and self.debug:
            time.sleep(0.15)
            try:
                j = [round(float(x)) for x in self.dsr.dsr._cur_j()]
            except Exception:
                j = "?"
            print("  [move] 명령 TCP=(%.0f, %.0f, %.0f)  실제 TCP=(%.0f, %.0f, %.0f)  J=%s" % (*posx[:3], *f.vis.tool[:3], j), flush=True)

    def rotate_j6(self, deg):
        self._both("rotate_j6", deg)

    def set_do(self, pin, value):
        self.dsr.set_do(pin, value)
        self.viz.set_do(pin, value)

    def wait(self, sec, tick=None):
        t = threading.Thread(target=self.viz.wait, args=(sec, tick), daemon=True)
        t.start()
        self.dsr.wait(sec)
        t.join()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--step", help="pour_batter | add_filling | cover_filling | wait_bake | flip_mold | pick_to_bag")
    ap.add_argument("--flavor", default="redbean", help="앙금: redbean | cream | choco, 칸별이면 쉼표로 (예: redbean,choco)")
    ap.add_argument("--cells", type=int, default=0, help="한 판에 만들 개수 (0 = poses.yaml 의 cells.count)")
    ap.add_argument("--vel-x", type=float, default=150, help="직선 속도 mm/s")
    ap.add_argument("--vel-j", type=float, default=40, help="관절 속도 deg/s")
    ap.add_argument("--tcp-z", type=float, default=117.0, help="플랜지→손가락 끝 거리(mm). 공식 RH-P12-RN-A 메시 기준 약 117")
    ap.add_argument("--table", default="120,1080,275,275")
    ap.add_argument("--poses", default=str(PKG / "config" / "poses.yaml"))
    ap.add_argument("--loop", action="store_true")
    args = ap.parse_args()

    poses = load_poses(args.poses)
    poses = {**poses, "gripper": {**poses.get("gripper", {}), "type": "mock"},
             "speed": {"vel_j": args.vel_j, "acc_j": args.vel_j, "vel_x": args.vel_x, "acc_x": args.vel_x}}
    from boong_robot.menu import parse_menu
    args.flavor = parse_menu(args.flavor)
    if isinstance(args.flavor, list) and not args.cells:
        args.cells = len(args.flavor)          # 칸별 메뉴 개수 = 칸 수
    args.cells = args.cells or int(poses.get("cells", {}).get("count", 1))
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

    calls = dry_run(poses, run)
    print("\n".join(summary(calls)))
    problems = validate(calls)
    if problems:
        print("\n⚠️ 거부: 좌표를 먼저 고치세요\n" + "\n".join(problems))
        sys.exit(2)

    import rclpy
    import DR_init
    from std_msgs.msg import Int32

    rclpy.init()
    node = rclpy.create_node("boong_sim", namespace=ROBOT_ID)
    DR_init.__dsr__id, DR_init.__dsr__model, DR_init.__dsr__node = ROBOT_ID, ROBOT_MODEL, node
    from DSR_ROBOT2 import ROBOT_MODE_AUTONOMOUS, get_current_posx, set_robot_mode

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    print(f"poses.yaml 좌표 = 손가락 끝 기준 (플랜지 +z {args.tcp_z} mm 오프셋을 명령에서 보정)")

    from arm_model import Vis, layout_warnings, parse_table
    from boong_robot.backends import DsrBackend
    from boong_robot.menu import MenuBungeoppang as Bungeoppang
    from viz_demo import DemoGripper, DemoScene, Publisher, RvizBackend

    vis = Vis(poses)
    vis.show_arm = False
    vis.n_active = args.cells
    vis.table = parse_table(args.table)
    vis.warn = layout_warnings(poses, vis.table, calls)
    for w in vis.warn:
        print("⚠️ 배치:", w)

    pub = Publisher(rclpy.create_node("boong_env_viz"), vis)
    # 예전 실행(예: 틀 3개)에서 남은 마커를 먼저 전부 지운다 → 지금 설정한 틀 개수만 보이게
    from visualization_msgs.msg import Marker, MarkerArray
    clear = Marker(); clear.action = Marker.DELETEALL; clear.ns = "boong"; clear.header.frame_id = "base_link"
    for _ in range(3):
        pub.pub.publish(MarkerArray(markers=[clear])); time.sleep(0.1)
    viz = StateOnlyViz(vis, pub, poses, speed=1.0)
    dsr = FingertipBackend(DsrBackend(poses["speed"], log=print, tool=None), args.tcp_z)
    both = BothBackend(dsr, viz)

    class SimGripper(DemoGripper):
        """환경(잡은 물건·짜기) 상태 + 실제 그리퍼 모델(position_cmd)을 같이 구동."""

        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self.cmd_pub = node.create_publisher(Int32, f"/{ROBOT_ID}/gripper/position_cmd", 10)

        def _cmd(self, pos):
            self.cmd_pub.publish(Int32(data=int(pos)))

        def grip(self):
            self._cmd(700); super().grip()

        def hold(self, pos):
            self._cmd(pos); super().hold(pos)

        def squeeze(self, pos, sec, back_to):
            self._cmd(pos)
            threading.Timer(sec, lambda: self._cmd(back_to)).start()
            super().squeeze(pos, sec, back_to)

        def release(self):
            self._cmd(0); super().release()

    bot = Bungeoppang(both, poses, on_step=lambda s, d: (print(f"[{s}] {d}"), setattr(vis, "label", f"{s}: {d}")),
                      scene=DemoScene(vis, poses))
    bot.gripper = SimGripper(both, poses.get("gripper", {}), vis, poses)

    # 환경 마커는 실제 로봇 TF(link_6)를 따라간다 (별도 노드 + 스레드)
    tf_node = rclpy.create_node("boong_tf_follow")
    from rclpy.executors import SingleThreadedExecutor
    tf_exec = SingleThreadedExecutor()      # 전역 executor 는 DSR_ROBOT2 가 쓰므로 따로 둔다
    tf_exec.add_node(tf_node)
    threading.Thread(target=tf_exec.spin, daemon=True).start()
    follower = TfFollower(vis, pub, poses, args.tcp_z, tf_node, rclpy)
    both.follower = follower
    threading.Thread(target=follower.run, daemon=True).start()
    t0 = time.time()
    while not follower.ok and time.time() - t0 < 15:
        time.sleep(0.2)
    print("TF 추적:", "OK" if follower.ok else "실패(link_6 TF 없음)", [round(x, 1) for x in vis.tool])

    try:
        while True:
            run(bot)
            if args.step:
                bot.home()
            print("끝")
            if not args.loop:
                break
    except KeyboardInterrupt:
        print("중단")
    finally:
        follower.stop = True
        time.sleep(0.2)
        tf_exec.shutdown(timeout_sec=1.0)   # TF 스레드를 먼저 멈춰야 종료 때 Traceback 이 안 남
        rclpy.shutdown()


if __name__ == "__main__":
    main()
