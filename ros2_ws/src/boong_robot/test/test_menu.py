"""고객 메뉴(칸별 앙금: 팥 / 생크림 / 초코) — 반죽 → 앙금 → 반죽 순서와 통 선택 검사."""
from pathlib import Path

import pytest

from boong_robot.backends import MockBackend
from boong_robot.menu import MenuBungeoppang, parse_menu
from boong_robot.poses import approach, load_poses

CLICKER = Path(__file__).resolve().parents[1] / "config" / "poses.clicker.example.yaml"
POSES = {**load_poses(CLICKER), "gripper": {"type": "mock", "wait_sec": 0}}


def _bot():
    b = MockBackend(log=lambda *_: None, speed=0)
    steps = []
    return b, steps, MenuBungeoppang(b, POSES, on_step=lambda s, d: steps.append((s, d)))


def _bottles_visited(b):
    """명령 목록에서 통을 집으러 간 순서 (통 좌표로 직접 movel 한 곳)."""
    spots = {tuple(POSES[n]["posx"]): n for n in ("batter_bottle", "redbean_bottle", "cream_bottle", "choco_bottle")}
    out = []
    for c in b.calls:
        if c[0] == "movel" and tuple(c[1]) in spots:
            name = spots[tuple(c[1])]
            if not out or out[-1] != name:
                out.append(name)
    return out        # 집기·내려놓기는 같은 통이 연달아 찍혀 하나로 합쳐진다


def test_order_is_batter_then_filling_then_batter():
    b, steps, bot = _bot()
    bot.make_batch("redbean", 2)
    assert _bottles_visited(b) == ["batter_bottle", "redbean_bottle", "batter_bottle"]
    kinds = [d for s, d in steps if s in ("batter", "filling")]
    assert kinds[0] == "반죽 짜는 중" and kinds[1].startswith("앙금 짜는 중") and kinds[2] == "반죽으로 덮는 중"


@pytest.mark.parametrize("flavor,bottle", [("redbean", "redbean_bottle"), ("cream", "cream_bottle"), ("choco", "choco_bottle")])
def test_each_menu_uses_its_own_bottle(flavor, bottle):
    b, _, bot = _bot()
    bot.make_batch(flavor, 1)
    assert _bottles_visited(b) == ["batter_bottle", bottle, "batter_bottle"]


def test_per_cell_menu_picks_each_bottle_once_and_fills_only_its_cell():
    b, _, bot = _bot()
    bot.make_batch(["redbean", "choco"], 2)
    assert _bottles_visited(b) == ["batter_bottle", "redbean_bottle", "choco_bottle", "batter_bottle"]
    # 팥은 0번 칸 위에서만, 초코는 1번 칸 위에서만 짠다
    d0 = POSES["mold_dispense"]["posx"]
    pitch = POSES["cells"]["pitch_mm"]
    i_red = b.calls.index(("movel", tuple(POSES["redbean_bottle"]["posx"])))
    i_choco = b.calls.index(("movel", tuple(POSES["choco_bottle"]["posx"])))
    red_part = b.calls[i_red:i_choco]
    ys = {round(c[1][1]) for c in red_part if c[0] == "movel" and round(c[1][0]) == d0[0] and round(c[1][2]) == d0[2]}
    assert ys == {d0[1]}                       # 팥: 0번 칸 y 만
    batter = ("movel", tuple(POSES["batter_bottle"]["posx"]))
    i_cover = b.calls.index(batter, i_choco)  # 초코 다음 = 덮기 반죽통 집기
    choco_part = b.calls[i_choco:i_cover]
    ys = {round(c[1][1]) for c in choco_part if c[0] == "movel" and round(c[1][0]) == d0[0] and round(c[1][2]) == d0[2]}
    assert d0[1] + pitch in ys and d0[1] not in ys   # 초코: 1번 칸 y 만


def test_same_menu_in_two_cells_picks_bottle_once():
    b, _, bot = _bot()
    bot.add_filling(["cream", "cream"], (0, 1))
    assert _bottles_visited(b) == ["cream_bottle"]


def test_bad_menu_is_rejected_before_any_motion():
    b, _, bot = _bot()
    with pytest.raises(ValueError):
        bot.make_batch(["redbean", "mint"], 2)
    with pytest.raises(ValueError):
        bot.make_batch(["redbean"], 2)          # 칸 2개에 메뉴 1개
    assert b.calls == []


def test_parse_menu():
    assert parse_menu("choco") == "choco"
    assert parse_menu("redbean, choco") == ["redbean", "choco"]
    assert approach is not None


def test_plate_flip_closes_bakes_both_sides_then_opens_and_homes():
    from boong_robot import plate_frame
    b, steps, bot = _bot()
    bot.flip_mold()
    labels = [d for s, d in steps if s == "baking"]
    assert labels == ["뚜껑 닫는 중", "앞면 굽는 중", "틀 뒤집는 중", "뒷면 굽는 중", "틀 다시 뒤집는 중", "뚜껑 여는 중"]
    fr, grip = POSES["frame"], POSES["mold_handle"]["posx"]
    moves = [list(c[1]) for c in b.calls if c[0] == "movel"]
    assert plate_frame.stack_pose(grip, fr, 180.0) in moves     # 묶음째 로봇 쪽으로 넘김
    assert plate_frame.grip_pose(grip, fr, 180.0) in moves      # 뚜껑 닫힌 자리
    assert b.calls[0][0] == "home" and b.calls[-1][0] == "home" # 앞뒤 홈 → 손목 감김 풀기
    assert all(m[2] >= 30 for m in moves)                       # 테이블 위
