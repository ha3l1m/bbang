"""로봇·ROS 없이 동작 순서 검증: pytest ros2_ws/src/boong_robot/test"""
from pathlib import Path

import pytest

from boong_robot.backends import MockBackend
from boong_robot.gripper import Gripper
from boong_robot.motions import Bungeoppang
from boong_robot.poses import above, approach, load_poses, spun

POSES = load_poses(Path(__file__).parents[1] / "config" / "poses.yaml")
MAT = POSES["materials"]


class FakeCaller:
    """그리퍼 서비스/토픽 호출을 백엔드 기록(calls)에 섞어 넣어 순서를 검증한다."""

    def __init__(self, backend):
        self.b = backend

    def call(self, name):
        self.b.calls.append(("grip_srv", name))

    def set_position(self, pos):
        self.b.calls.append(("pos", pos))


def make():
    b = MockBackend(log=lambda *_: None, speed=0)
    steps = []
    bot = Bungeoppang(b, {**POSES, "gripper": {"type": "mock"}}, on_step=lambda s, d: steps.append(s))
    bot.gripper = Gripper(b, {"type": "ros_service"}, caller=FakeCaller(b))
    return bot, b, steps


def movels(calls):
    return [c[1] for c in calls if c[0] == "movel"]


def f(pose):
    return tuple(round(v, 1) for v in pose)


def test_web_steps_in_order():
    bot, _, steps = make()
    bot.make_one("redbean")
    dedup = [x for i, x in enumerate(steps) if i == 0 or steps[i - 1] != x]
    assert dedup == ["batter", "filling", "baking", "packed"]


def test_starts_and_ends_at_home():
    bot, b, _ = make()
    bot.make_one("redbean")
    assert b.calls[0][0] == "home" and b.calls[-1][0] == "home"


def test_dispense_inverts_then_restores_bottle():
    bot, b, _ = make()
    bot.pour_batter()
    assert [c for c in b.calls if c[0] == "rotate_j6"] == [("rotate_j6", 180), ("rotate_j6", -180)]


def test_squeeze_happens_while_bottle_is_upside_down():
    bot, b, _ = make()
    bot.pour_batter()
    i, j = b.calls.index(("rotate_j6", 180)), b.calls.index(("rotate_j6", -180))
    assert ("pos", MAT["batter"]["squeeze"]) in b.calls[i:j]
    assert ("pos", MAT["batter"]["squeeze"]) not in b.calls[:i] + b.calls[j:]


def test_lowers_over_mold_with_rotated_wrist():
    bot, b, _ = make()
    bot.pour_batter()
    i = b.calls.index(("rotate_j6", 180))
    nxt = next(c for c in b.calls[i + 1:] if c[0] == "movel")
    assert nxt[1] == f(spun(POSES["mold_dispense"]["posx"], 180))


def test_bottle_is_approached_from_the_side():
    bot, b, _ = make()
    bot.pour_batter()
    first = movels(b.calls)[0]
    bottle = POSES["batter_bottle"]["posx"]
    assert first[1] == bottle[1] + POSES["approach_z"]  # 툴이 -y를 향하므로 +y 쪽에서 접근
    assert first[2] == bottle[2]  # 높이는 그대로 (위에서 내려오지 않는다)


def test_squeezes_in_order_batter_filling_cover():
    bot, b, _ = make()
    bot.make_one("redbean")
    squeezes = {MAT[k]["squeeze"] for k in MAT}
    positions = [c[1] for c in b.calls if c[0] == "pos" and c[1] in squeezes]
    assert positions == [MAT["batter"]["squeeze"], MAT["redbean"]["squeeze"], MAT["batter"]["squeeze"]]


def test_thicker_material_is_squeezed_harder_and_longer():
    assert MAT["redbean"]["squeeze"] >= MAT["batter"]["squeeze"]
    assert MAT["redbean"]["squeeze_sec"] >= MAT["batter"]["squeeze_sec"]
    for m in MAT.values():
        assert m["squeeze"] > m["hold"]


def test_cream_uses_cream_bottle():
    bot, b, _ = make()
    bot.add_filling("cream")
    assert f(POSES["cream_bottle"]["posx"]) in movels(b.calls)
    assert f(POSES["redbean_bottle"]["posx"]) not in movels(b.calls)


def test_flip_closes_lids_then_reopens_without_lifting():
    bot, b, _ = make()
    bot.flip_mold()
    assert [c for c in b.calls if c[0] == "rotate_j6"] == [("rotate_j6", 180), ("rotate_j6", -180)]
    handle = POSES["mold_handle"]["posx"]
    zs = [c[1][2] for c in b.calls if c[0] == "movel"]
    assert max(zs) <= handle[2]           # 커튼봉은 받침대에 걸려 있어 들어올리지 않는다
    # 닫힌 채 굽는 시간은 두 회전 사이
    i, j = b.calls.index(("rotate_j6", 180)), b.calls.index(("rotate_j6", -180))
    assert any(c[0] == "wait" and c[1] == POSES["flip_bake_sec"] for c in b.calls[i:j])


def test_flip_lift_option_lifts_whole_rod():
    bot, b, _ = make()
    bot.p = {**bot.p, "flip_lift_mm": 80}
    bot.flip_mold()
    handle = POSES["mold_handle"]["posx"]
    assert max(c[1][2] for c in b.calls if c[0] == "movel") == handle[2] + 80


def test_handle_is_approached_along_tool_axis():
    bot, b, _ = make()
    bot.flip_mold()
    first = movels(b.calls)[0]
    handle = POSES["mold_handle"]["posx"]
    assert first[1] == handle[1] + POSES["approach_z"] and first[2] == handle[2]


def test_full_sequence_gripper_usage():
    bot, b, _ = make()
    bot.make_one("redbean")
    assert b.calls.count(("grip_srv", "close")) == 2  # 틀 손잡이, 붕어빵
    holds = [c[1] for c in b.calls if c[0] == "pos" and c[1] == MAT["batter"]["hold"]]
    assert len(holds) >= 3
    assert b.calls[[i for i, c in enumerate(b.calls) if c[0] == "grip_srv"][-1]] == ("grip_srv", "open")


def test_approach_equals_above_for_tool_pointing_down():
    p = [400, 0, 100, 0, 180, 0]
    assert [round(v, 3) for v in approach(p, 100)] == above(p, 100)


def test_approach_retreats_sideways_for_horizontal_tool():
    p = [470, 180, 110, -90, 90, 0]
    assert approach(p, 100)[:3] == [470.0, 280.0, 110.0]


def test_unknown_flavor():
    bot, _, _ = make()
    with pytest.raises(ValueError):
        bot.add_filling("choco")


def make3():
    b = MockBackend(log=lambda *_: None, speed=0)
    poses = {**POSES, "cells": {"count": 3, "pitch_mm": 110}, "gripper": {"type": "mock"}}
    bot = Bungeoppang(b, poses, on_step=lambda *_: None)
    bot.gripper = Gripper(b, {"type": "ros_service"}, caller=FakeCaller(b))
    return bot, b


def test_batch_of_three_visits_each_cell_with_one_bottle_pickup_per_material():
    bot, b = make3()
    bot.make_batch("redbean", 3)
    dst = POSES["mold_dispense"]["posx"]
    targets = [c[1] for c in b.calls if c[0] == "movel" and c[1][5] == dst[5] + 180 and c[1][2] == dst[2]]
    assert sorted({t[1] for t in targets}) == [dst[1], dst[1] + 110, dst[1] + 220]
    assert len(targets) == 9                                   # 반죽·앙금·덮기 × 3칸
    assert [c for c in b.calls if c[0] == "rotate_j6"].count(("rotate_j6", 180)) == 10   # 짜기 9 + 뚜껑 1
    assert [c for c in b.calls if c[0] == "rotate_j6"].count(("rotate_j6", -180)) == 10
    assert b.calls.count(("grip_srv", "close")) == 1 + 3      # 봉 1 + 붕어빵 3
    bag = [c for c in b.calls if c[0] == "movel" and c[1][:3] == tuple(POSES["bag_drop"]["posx"][:3])]
    assert len(bag) == 3


def test_batch_steps_and_limits():
    bot, b = make3()
    steps = []
    bot.on_step = lambda s, d: steps.append(s)
    bot.make_batch("cream", 2)
    assert [x for i, x in enumerate(steps) if i == 0 or steps[i - 1] != x] == ["batter", "filling", "baking", "packed"]
    with pytest.raises(ValueError):
        bot.make_batch("cream", 4)
    with pytest.raises(ValueError):
        bot.make_batch("cream", 0)


# ---- 사다리(뚜껑 틀) 반원 뒤집기: config/poses.clicker.example.yaml 의 frame 설정 ----
import math

CLICKER = load_poses(Path(__file__).parents[1] / "config" / "poses.clicker.example.yaml")


def make_frame():
    b = MockBackend(log=lambda *_: None, speed=0)
    return Bungeoppang(b, CLICKER), b


def frame_moves(calls):
    return [c[1] for c in calls if c[0] == "movel"]


def test_frame_arc_is_semicircle_about_hinge_rod():
    bot, b = make_frame()
    bot.flip_mold()
    f, rod = CLICKER["frame"], CLICKER["mold_handle"]["posx"]
    d, px = f["span_mm"], rod[0] + f["span_mm"]
    arc = [q for q in frame_moves(b.calls) if abs(math.hypot(q[0] - px, q[2] - rod[2]) - d) < 0.2]
    assert len(arc) >= 2 * f["steps"]                      # 닫기·열기 각 steps 개
    assert max(q[2] for q in arc) == pytest.approx(rod[2] + d, abs=0.2)
    xs = [q[0] for q in arc]
    assert min(xs) == pytest.approx(rod[0]) and max(xs) == pytest.approx(rod[0] + 2 * d)


def test_frame_tool_rolls_with_the_rail():
    bot, b = make_frame()
    bot.flip_mold()
    assert not [c for c in b.calls if c[0] == "rotate_j6"]
    rod, f = CLICKER["mold_handle"]["posx"], CLICKER["frame"]
    d, px = f["span_mm"], rod[0] + f["span_mm"]
    for q in frame_moves(b.calls):
        if abs(math.hypot(q[0] - px, q[2] - rod[2]) - d) > 0.2:
            continue
        th = math.degrees(math.atan2(q[2] - rod[2], px - q[0])) if q[0] <= px else 180 - math.degrees(math.atan2(q[2] - rod[2], q[0] - px))
        assert q[5] - rod[5] == pytest.approx(f["roll_sign"] * th, abs=0.3)   # 위치 각도 = 툴 롤


def test_frame_releases_on_far_hook_bakes_then_regrabs_and_reopens():
    bot, b = make_frame()
    calls = []
    orig_rel, orig_grip = bot.gripper.release, bot.gripper.grip
    bot.gripper.release = lambda: (calls.append("release"), orig_rel())
    bot.gripper.grip = lambda: (calls.append("grip"), orig_grip())
    b.move_line = (lambda f: lambda x: (calls.append(("movel", tuple(x))), f(x)))(b.move_line)
    b.wait = (lambda f: lambda s: (calls.append(("bake", s)) if s == CLICKER["flip_bake_sec"] else None, f(s)))(b.wait)
    bot.flip_mold()
    rod, d = CLICKER["mold_handle"]["posx"], CLICKER["frame"]["span_mm"]
    bake = calls.index(("bake", CLICKER["flip_bake_sec"]))
    assert calls[:bake].count("grip") == 1 and "release" in calls[calls.index("grip"):bake]   # 닫은 뒤 놓고 굽는다
    assert any(isinstance(c, tuple) and c[0] == "movel" and abs(c[1][0] - (rod[0] + 2 * d)) < .1 for c in calls[:bake])
    assert calls[bake:].count("grip") == 1                                                    # 다시 잡아서 연다
    last = [c for c in calls if isinstance(c, tuple) and c[0] == "movel"][-1][1]
    assert last[0] == rod[0] and last[5] == rod[5]


def test_frame_batch_stays_inside_limits():
    b = MockBackend(log=lambda *_: None, speed=0)
    Bungeoppang(b, CLICKER).make_batch("redbean", 3)
    pts = frame_moves(b.calls)
    assert min(q[2] for q in pts) >= 70 and max(math.hypot(q[0], q[1]) for q in pts) < 900


# ---- DRL 그리퍼 (RH-P12-RN-A Modbus over Tool Flange) ----
def test_drl_modbus_frames():
    from boong_robot import drl_gripper as d
    assert d.crc16([0x01, 0x03, 0x00, 0x00, 0x00, 0x0A]) == [0xC5, 0xCD]      # 표준 Modbus 예제 벡터
    assert d.fc06_torque_on()[:6] == [1, 0x06, 0x01, 0x00, 0x00, 0x01]
    f = d.fc16_goal_position(700)
    assert f[:11] == [1, 0x10, 0x01, 0x1A, 0, 2, 4, 0x02, 0xBC, 0, 0] and len(f) == 13
    assert d.fc16_goal_position(9999)[7:9] == [0x02, 0xBC] and d.fc16_goal_position(-5)[7:9] == [0, 0]   # 0~700 으로 제한
    code = d.make_drl(350, 1.0)
    assert "flange_serial_open(baudrate=57600" in code and code.rstrip().endswith("flange_serial_close()")
    assert code.count("flange_serial_write") == 2


def test_gripper_drl_type_sends_positions_via_caller():
    b = MockBackend(log=lambda *_: None, speed=0)
    sent = []

    class Fake:
        def call(self, name): sent.append(name)
        def set_position(self, pos): sent.append(pos)

    g = Gripper(b, {"type": "drl", "wait_sec": 0.1}, caller=Fake())
    g.hold(380); g.squeeze(620, 2.0, 380); g.grip(); g.release()
    assert sent == [380, 620, 380, "close", "open"]


def test_dsr_backend_selects_registered_tool_and_tcp(monkeypatch):
    import sys, types
    calls = []
    fake = types.ModuleType("DSR_ROBOT2")
    for n in ("get_current_posj", "movej", "movel", "posj", "posx", "set_digital_output", "set_velj", "set_accj", "set_velx", "set_accx"):
        setattr(fake, n, lambda *a, **k: None)
    fake.set_tool = lambda n: calls.append(("tool", n))
    fake.set_tcp = lambda n: calls.append(("tcp", n))
    monkeypatch.setitem(sys.modules, "DSR_ROBOT2", fake)
    from boong_robot.backends import DsrBackend
    DsrBackend({"vel_j": 30}, log=lambda *_: None)
    assert calls == []                                             # 설정 없으면 건드리지 않는다
    DsrBackend({"vel_j": 30}, log=lambda *_: None, tool={"name": "g", "tcp": "t"})
    assert calls == [("tool", "g"), ("tcp", "t")]
