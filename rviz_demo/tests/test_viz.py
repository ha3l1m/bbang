"""ROS 없이 검증: arm_model(순수 계산) + viz_demo(가짜 rclpy/visualization_msgs 로 실행)
   cd rviz_demo && python3 -m pytest -q tests
"""
import math
import sys
import types
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
PKG = HERE.parent / "ros2_ws" / "src" / "boong_robot"
sys.path.insert(0, str(PKG))

import arm_model as am  # noqa: E402
from boong_robot.poses import load_poses  # noqa: E402

CLICKER = PKG / "config" / "poses.clicker.example.yaml"
POSES = load_poses(CLICKER)
N = int(POSES["cells"]["count"])   # 틀(칸) 수 — 지금은 2개


def test_home_pose_is_reachable_and_elbow_is_up():
    a = am.arm_points([370, 0, 440, 0, 180, 0])
    assert a["reach"] and a["elbow"][2] > a["shoulder"][2]


def test_every_station_is_reachable():
    for name in ["batter_bottle", "redbean_bottle", "cream_bottle", "mold_dispense", "mold_handle", "bread_pick", "bag_drop"]:
        assert am.arm_points(POSES[name]["posx"])["reach"], name
    assert am.arm_points([470, 0, 240, -90, 90, 180])["reach"]


def test_bottle_axis_flips_with_j6():
    up = am.tool_axes([470, 0, 240, -90, 90, 0])[0]
    down = am.tool_axes([470, 0, 240, -90, 90, 180])[0]
    assert [round(-c) for c in up] == [0, 0, 1]      # c=0: 노즐 위
    assert [round(-c) for c in down] == [0, 0, -1]   # c=180: 노즐 아래


def test_quat_z_to():
    assert am.quat_z_to([0, 0, 1]) == pytest.approx((0, 0, 0, 1))
    assert am.quat_z_to([0, 0, -1]) == pytest.approx((1, 0, 0, 0))


def test_finger_offset_closes_then_squeezes():
    assert am.finger_offset(0, True) > am.finger_offset(380, True) > am.finger_offset(620, True)


# ---------------- 가짜 ROS 로 viz_demo 전체 실행 ----------------
class _Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _Marker:
    CUBE, SPHERE, CYLINDER, LINE_STRIP, LINE_LIST, SPHERE_LIST, TEXT_VIEW_FACING = 1, 2, 3, 4, 5, 7, 9
    ADD, DELETE = 0, 2

    def __init__(self):
        self.header = _Obj(frame_id="", stamp=None)
        self.pose = _Obj(position=_Obj(x=0, y=0, z=0), orientation=_Obj(x=0, y=0, z=0, w=1))
        self.scale, self.color = _Obj(x=0, y=0, z=0), _Obj(r=0, g=0, b=0, a=0)
        self.points, self.text, self.id, self.ns, self.type, self.action = [], "", 0, "", 0, 0


class _Node:
    def __init__(self, sink):
        self.sink = sink

    def create_publisher(self, *_a):
        return _Obj(publish=lambda arr: self.sink.append(arr))

    def get_clock(self):
        return _Obj(now=lambda: _Obj(to_msg=lambda: 0))


@pytest.fixture()
def demo(monkeypatch):
    frames = []
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.shutdown = lambda: None
    rclpy.create_node = lambda _n: _Node(frames)
    vm = types.ModuleType("visualization_msgs"); vm.msg = types.ModuleType("visualization_msgs.msg")
    vm.msg.Marker = _Marker; vm.msg.MarkerArray = lambda: _Obj(markers=[])
    gm = types.ModuleType("geometry_msgs"); gm.msg = types.ModuleType("geometry_msgs.msg")
    gm.msg.Point = lambda x, y, z: (x, y, z)
    for n, m in [("rclpy", rclpy), ("visualization_msgs", vm), ("visualization_msgs.msg", vm.msg), ("geometry_msgs", gm), ("geometry_msgs.msg", gm.msg)]:
        monkeypatch.setitem(sys.modules, n, m)
    sys.modules.pop("viz_demo", None)
    import viz_demo
    monkeypatch.setattr(viz_demo.time, "sleep", lambda s: None)
    monkeypatch.setattr(viz_demo, "FPS", 6)
    return viz_demo, frames


def _by_id(arr):
    return {m.id: m for m in arr.markers if m.action == _Marker.ADD}


def test_full_run_shows_inverted_bottle_pouring_over_mold(demo, monkeypatch):
    viz_demo, frames = demo
    monkeypatch.setattr(sys, "argv", ["viz_demo", "--speed", "50", "--poses", str(CLICKER), "--startup-wait", "0", "--flavor", "redbean", "--exit"])
    viz_demo.main()

    mold_top = am.TOP_Z / 1000
    pours = [_by_id(f) for f in frames if 60 in _by_id(f)]
    assert pours, "짜는 장면이 한 번도 안 나옴"
    for ms in pours:  # 짜는 동안 통은 뒤집혀 있고(노즐 캡이 몸통보다 아래), 노즐 끝이 틀 윗면 근처
        held = [i for i in (10, 11, 12) if ms[i].pose.orientation.x > 0.99]
        assert len(held) == 1
        body, cap = ms[held[0]], ms[held[0] + 10]
        assert cap.pose.position.z < body.pose.position.z
        assert 0 < cap.pose.position.z - cap.scale.z / 2 - mold_top < 0.04
    # 병이 뒤집힌 채 틀 위(x=470,y=0)에서 짜이는지
    last = pours[-1]
    assert abs(last[61].pose.position.x - POSES["mold_dispense"]["posx"][0] / 1000) < 1e-6
    # 마지막: 붕어빵이 봉투 위치, 아무것도 안 들고 있음
    final = _by_id(frames[-1])
    assert abs(final[140].pose.position.x - POSES["bag_drop"]["posx"][0] / 1000) < 1e-6
    assert 10 in final and 6 in final


def test_flip_step_closes_then_reopens_lids(demo, monkeypatch):
    viz_demo, frames = demo
    monkeypatch.setattr(sys, "argv", ["viz_demo", "--speed", "50", "--poses", str(CLICKER), "--startup-wait", "0", "--step", "flip_mold", "--exit"])
    viz_demo.main()
    lids = [_by_id(f)[120].pose.orientation for f in frames if 120 in _by_id(f)]
    assert max(abs(o.y) for o in lids) > 0.99            # 180° 닫힘
    assert abs(lids[-1].w) > 0.99                         # 다시 열려서 끝남


def test_batch_of_three_all_end_in_bag(demo, monkeypatch):
    viz_demo, frames = demo
    monkeypatch.setattr(sys, "argv", ["viz_demo", "--speed", "50", "--poses", str(CLICKER), "--startup-wait", "0", "--count", str(N), "--exit"])
    viz_demo.main()
    final = _by_id(frames[-1])
    bag_x = POSES["bag_drop"]["posx"][0] / 1000
    assert all(abs(final[140 + i].pose.position.x - bag_x) < 1e-6 for i in range(N))
    assert {100 + i for i in range(N)} <= set(final)     # 클리커 칸이 그려짐
    assert {150 + i for i in range(N)} <= set(final)     # 붕어빵 음각이 그려짐
    assert 100 + N not in final                          # 칸은 정확히 N개


def test_clicker_layout_fits_the_estimated_table_and_detects_overhang():
    from boong_robot.backends import MockBackend
    from boong_robot.menu import MenuBungeoppang as Bungeoppang
    b = MockBackend(log=lambda *_: None, speed=0)
    Bungeoppang(b, POSES).make_batch("redbean", N)
    narrow = am.parse_table("120,1080,275,275")
    assert am.layout_warnings(POSES, narrow, b.calls) == []
    far = {**POSES, "batter_bottle": {"posx": [350, -300, 105, 0, 90, 0]}}   # 통을 테이블 밖에 두면 잡아낸다
    assert any("batter_bottle" in x for x in am.layout_warnings(far, narrow, b.calls))
    tiny = am.parse_table("120,1080,120,120")
    assert am.layout_warnings(POSES, tiny, b.calls)


def test_table_marker_present():
    v = am.Vis(POSES)
    ids = [m["id"] for m in am.build_markers(v)]
    assert 80 in ids and 81 not in ids


def test_plate_lid_flips_over_hinge_and_reopens(demo, monkeypatch):
    """철판 뚜껑 판(마커 131)이 경첩 둘레로 반원을 그려 아래 판 위로 넘어갔다가 다시 열린다."""
    from boong_robot import plate_frame
    viz_demo, frames = demo
    monkeypatch.setattr(sys, "argv", ["viz_demo", "--speed", "50", "--poses", str(CLICKER), "--startup-wait", "0", "--step", "flip_mold", "--exit"])
    viz_demo.main()
    fr, grip = POSES["frame"], POSES["mold_handle"]["posx"]
    px, pz = (c / 1000 for c in plate_frame.pivot(grip, fr))
    half_depth = fr["plate_mm"][0] / 2 / 1000
    lid = [_by_id(f)[131].pose.position for f in frames if 131 in _by_id(f)]
    assert max(p.z for p in lid) > pz + 0.9 * half_depth          # 반원 꼭대기까지 들림
    assert max(p.x for p in lid) > px + 0.9 * half_depth          # 경첩 건너편(아래 판 위)까지 넘어감
    assert abs(lid[-1].x - (px - half_depth)) < 0.003             # 다시 열려 로봇 쪽으로 펼쳐짐


def test_plate_closed_grip_is_35mm_higher_across_the_hinge():
    from boong_robot import plate_frame
    fr, grip = POSES["frame"], POSES["mold_handle"]["posx"]
    closed = plate_frame.grip_pose(grip, fr, 180.0)
    assert abs(closed[0] - (grip[0] + 2 * fr["grip_r_mm"])) < 1e-6   # 경첩 건너편
    assert abs(closed[2] - (grip[2] + fr["closed_mm"])) < 1e-6        # 닫힌 높이 35mm 만큼 위
    assert abs(closed[5] - (grip[5] + fr["roll_sign"] * 180)) < 1e-6  # 손목도 180°


def test_mold_plate_is_90_by_50_mm_with_engraved_fish():
    ms = {m["id"]: m for m in am.build_markers(am.Vis(POSES))}
    assert {130, 131, 132} <= set(ms)                                                        # 아래 판·뚜껑 판·받침 박스
    plate = ms[100]
    assert abs(plate["scale"][0] - 0.050) < 1e-9 and abs(plate["scale"][1] - 0.090) < 1e-9   # x 50 × y 90 mm
    assert {150, 160, 170} <= set(ms)                                                        # 몸통·꼬리·눈 음각
    assert abs(ms[101]["pos"][1] - ms[100]["pos"][1] - POSES["cells"]["pitch_mm"] / 1000) < 1e-9   # 틀 간격 (poses)


def test_closed_plates_turn_over_toward_robot_and_come_back(demo, monkeypatch):
    """뒤집기: 닫힌 묶음째 경첩 둘레로 넘어가 아래 판(130)이 로봇 쪽(-x)으로 갔다가, 다시 뒤집혀 제자리로."""
    from boong_robot import plate_frame
    viz_demo, frames = demo
    monkeypatch.setattr(sys, "argv", ["viz_demo", "--speed", "50", "--poses", str(CLICKER), "--startup-wait", "0", "--step", "flip_mold", "--exit"])
    viz_demo.main()
    fr, grip = POSES["frame"], POSES["mold_handle"]["posx"]
    px, _ = (c / 1000 for c in plate_frame.pivot(grip, fr))
    half_depth = fr["plate_mm"][0] / 2 / 1000
    bottom = [_by_id(f)[130].pose.position.x for f in frames if 130 in _by_id(f)]
    assert abs(bottom[0] - (px + half_depth)) < 0.003            # 처음: 경첩 +x 쪽
    assert min(bottom) < px - 0.9 * half_depth                   # 뒤집혀서 로봇 쪽으로 넘어감
    assert abs(bottom[-1] - (px + half_depth)) < 0.003           # 다시 뒤집혀 제자리
