"""붕어빵 동작 시퀀스 (ROS 의존성 없음 → 로봇 없이 테스트 가능).

재료(반죽·팥·슈크림)는 말랑한 소스통에 담겨 있다. 로봇은 통을 옆에서 집어 틀 위로 옮기고,
6축 손목(J6)을 180° 돌려 노즐을 아래로 향하게 한 뒤, 그리퍼를 더 닫아서 짠다.
세기는 재료마다 다르다 → poses.yaml 의 materials (점성이 높을수록 더 닫고 더 오래 누른다).

1 pour_batter    반죽 통 → 틀 위에서 뒤집어 짜기 → 다시 세워 제자리
2 add_filling    팥/슈크림 통으로 같은 동작
3 cover_filling  반죽 통으로 한 번 더 (앙금 덮기)
4 wait_bake      굽기 대기
5 flip_mold      뚜껑 틀의 봉 끝을 잡고 경첩 봉 둘레 반원(+J6 180°)으로 덮기 → 놓고 대기 → 반대로 열기
6 pick_to_bag    칸마다 붕어빵 집기 → 봉투 위 → 놓기   (make_batch(n): 한 판에 n개)
"""
import math
from typing import Callable

from .gripper import Gripper
from .poses import above, approach, cell_pose, spun

FILLINGS = ("redbean", "cream")

# 웹앱 단계 이름과 동일: received → batter → filling → baking → packed → ready
StepCb = Callable[[str, str], None]  # (step, detail)


def _noop(step: str, detail: str) -> None:
    pass


class Bungeoppang:
    def __init__(self, backend, poses: dict, on_step: StepCb = _noop, scene=None):
        self.b = backend
        self.p = poses
        self.on_step = on_step
        self.scene = scene  # RViz 마커 (없으면 생략)
        self.dz = poses.get("approach_z", 100)
        self.gripper = Gripper(backend, poses.get("gripper", {}))

    # ---- 공통 -------------------------------------------------------------
    def _x(self, name):
        return self.p[name]["posx"]

    @property
    def n_cells(self) -> int:
        return int(self.p.get("cells", {}).get("count", 1))

    def _cell(self, name: str, i: int):
        """name 좌표의 i번째 칸 (poses.yaml 의 cells.pitch_mm 간격)."""
        return cell_pose(self._x(name), i, float(self.p.get("cells", {}).get("pitch_mm", 0)))

    def home(self):
        self.b.home(self.p["home"]["posj"])

    def _down_up(self, target, action):
        """접근점(툴 반대쪽으로 물러난 점) → 목표 → action() → 다시 물러나기"""
        self.b.move_line(approach(target, self.dz))
        self.b.move_line(target)
        action()
        self.b.move_line(approach(target, self.dz))

    def _scene(self, state, flavor=None):
        if self.scene:
            self.scene.set_state(state, flavor)

    # ---- 소스통 짜기 ------------------------------------------------------
    def _dispense(self, material: str, cells=(0,)):
        """통을 한 번 집어서 cells 의 각 칸에 차례로 짜고 제자리에 돌려놓는다."""
        m = self.p["materials"][material]
        bottle = self._x(m["bottle"])
        lift = self.p.get("bottle_lift_mm", 150)
        clear = self.p.get("flip_clear_mm", 100)

        # 1) 통 집기: 옆에서 접근 → 가볍게 잡기 → 들어올리기
        self.gripper.release()
        self.b.move_line(approach(bottle, self.dz))
        self.b.move_line(bottle)
        self.gripper.hold(m["hold"])
        self.b.move_line(above(bottle, lift))
        for i in cells:
            dst = self._cell("mold_dispense", i)
            # 2) 칸 위로 가서 손목을 돌려 노즐을 아래로
            self.b.move_line(above(dst, clear))
            self.b.rotate_j6(180)
            down = spun(dst, 180)  # 손목이 180° 돈 자세 그대로 내려가기
            self.b.move_line(down)
            # 3) 짜기: 더 닫고 누르고, 다시 잡기 위치로
            self.gripper.squeeze(m["squeeze"], m["squeeze_sec"], back_to=m["hold"])
            # 4) 올라와서 통을 다시 세우기
            self.b.move_line(above(down, clear))
            self.b.rotate_j6(-180)
        # 5) 제자리에 내려놓기
        self.b.move_line(above(bottle, lift))
        self.b.move_line(bottle)
        self.gripper.release()
        self.b.move_line(approach(bottle, self.dz))

    # ---- 동작 -------------------------------------------------------------
    def pour_batter(self, cells=(0,)):
        self.on_step("batter", "반죽 짜는 중")
        self._dispense("batter", cells)
        self._scene("batter")

    def add_filling(self, flavor: str, cells=(0,)):
        if flavor not in FILLINGS:
            raise ValueError(f"모르는 맛: {flavor}")
        self.on_step("filling", "앙금 짜는 중")
        self._dispense(flavor, cells)
        self._scene("filling", flavor)

    def cover_filling(self, cells=(0,)):
        self.on_step("filling", "반죽으로 덮는 중")
        self._dispense("batter", cells)
        self._scene("covered")

    def wait_bake(self, sec=None):
        self.on_step("baking", "굽는 중")
        self.b.wait(self.p.get("bake_sec", 3.0) if sec is None else sec)
        self._scene("baked")

    def _arc(self, grip, closing: bool):
        """뚜껑 틀(봉 2개+뚜껑 가로대)을 가운데(경첩) 봉 축으로 반원 회전시키는 경로.
        grip = 열린 위치의 움직이는 봉 끝 좌표. 봉 방향(y)에 수직인 x–z 평면에서 반지름 span 의 원을 그리고,
        같은 각도만큼 툴(J6, 툴 z축=봉 축)을 같이 돌려 틀이 뒤틀리지 않게 한다."""
        f = self.p["frame"]
        d = float(f["span_mm"])
        n = int(f.get("steps", 6))
        sign = float(f.get("roll_sign", -1))
        for k in range(1, n + 1):
            th = 180.0 * (k if closing else n - k) / n
            r = math.radians(th)
            q = list(grip)
            q[0] = round(grip[0] + d - d * math.cos(r), 4)   # 경첩 봉(x0+d) 둘레
            q[2] = round(grip[2] + d * math.sin(r), 4)
            q[5] = grip[5] + sign * th
            self.b.move_line(q)

    def flip_mold(self):
        """뚜껑 닫기·열기.
        frame 설정이 있으면: 뚜껑 틀의 움직이는 봉 끝(mold_handle)을 잡고 고리에서 들어올려, 경첩(가운데) 고리 봉을 축으로
        반원(span_mm)을 그리며 J6 를 같이 180° 돌려 뚜껑을 붕어빵 틀 위에 덮고 세 번째 고리에 내려놓는다. 놓고 굽는 동안 대기,
        다시 잡아 반대로 돌려 연다.
        frame 이 없으면(예전 방식): 봉 끝을 잡고 J6 만 180° 돌렸다 되돌린다."""
        rod = self._x("mold_handle")
        f = self.p.get("frame")
        lift_mm = f.get("hook_lift_mm", 25) if f else self.p.get("flip_lift_mm", 0)
        self.on_step("baking", "뚜껑 닫는 중")
        self.gripper.release()
        self.b.move_line(approach(rod, self.dz))
        self.b.move_line(rod)
        self.gripper.grip()
        if lift_mm:
            self.b.move_line(above(rod, lift_mm))
        if f:
            self._arc(rod, closing=True)
            closed = self._closed_pose()          # 반원의 끝 = 세 번째 고리 위
            self.gripper.release()            # 뚜껑은 자기 무게로 덮여 있다
            self._scene("flipped")
            self.b.move_line(approach(closed, self.dz))
            self.on_step("baking", "뚜껑 닫고 굽는 중")
            self.b.wait(self.p.get("flip_bake_sec", 3.0))
            self.on_step("baking", "뚜껑 여는 중")
            self.b.move_line(closed)
            self.gripper.grip()
            if lift_mm:
                self.b.move_line(above(closed, lift_mm))
            self._arc(rod, closing=False)         # 반원의 끝 = 처음 고리 위
        else:
            self.b.rotate_j6(180)
            self._scene("flipped")
            self.on_step("baking", "뚜껑 닫고 굽는 중")
            self.b.wait(self.p.get("flip_bake_sec", 3.0))
            self.on_step("baking", "뚜껑 여는 중")
            self.b.rotate_j6(-180)
            if lift_mm:
                self.b.move_line(rod)
        self.gripper.release()
        self.b.move_line(approach(rod, self.dz))

    def _closed_pose(self):
        """뚜껑이 닫힌 뒤 움직이는 봉 끝 좌표 (열린 위치에서 경첩 건너편으로 2*span, J6 180°)."""
        f = self.p["frame"]
        rod = self._x("mold_handle")
        q = list(rod)
        q[0] = round(rod[0] + 2 * float(f["span_mm"]), 4)
        q[5] = rod[5] + float(f.get("roll_sign", -1)) * 180.0
        return q

    def pick_to_bag(self, cells=(0,)):
        self.on_step("packed", "봉투에 담는 중")
        bag = self._x("bag_drop")
        for i in cells:
            self.gripper.release()
            self._down_up(self._cell("bread_pick", i), self.gripper.grip)
            self.b.move_line(approach(bag, self.dz))
            self.b.move_line(bag)
            self.gripper.release()  # 떨어뜨리기
        self._scene("bagged")
        self.b.move_line(approach(bag, self.dz))

    # ---- 한 판(클리커 n개) / 1개 ------------------------------------------
    def make_batch(self, flavor: str, n: int = 1):
        if not 1 <= n <= self.n_cells:
            raise ValueError(f"한 번에 1~{self.n_cells}개 (poses.yaml cells.count), 요청 {n}")
        cells = tuple(range(n))
        self._scene("empty")
        self.home()
        self.pour_batter(cells)
        self.add_filling(flavor, cells)
        self.cover_filling(cells)
        self.wait_bake()
        self.flip_mold()
        self.pick_to_bag(cells)
        self.home()

    def make_one(self, flavor: str):
        self.make_batch(flavor, 1)
