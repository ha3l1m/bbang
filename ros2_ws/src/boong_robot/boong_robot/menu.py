"""고객 메뉴(칸별 앙금)용 붕어빵 시퀀스 — motions.Bungeoppang 을 확장한다 (motions.py 는 건드리지 않음).

순서는 그대로:  반죽 → 앙금(고객이 고른 메뉴: 팥 / 생크림 / 초코) → 반죽(덮기) → 굽기 → 뒤집기 → 봉투

    bot = MenuBungeoppang(backend, poses)
    bot.make_batch("redbean", 2)              # 2칸 모두 팥
    bot.make_batch(["redbean", "choco"], 2)   # 0번 칸 팥, 1번 칸 초코

같은 맛인 칸끼리 묶어서 그 통은 한 번만 집는다 (팥통 집기 → 0번 칸 → 내려놓기 → 초코통 집기 → 1번 칸 → 내려놓기).
맛 이름은 poses.yaml 의 materials 키와 같아야 한다 (예: materials.choco: {bottle: choco_bottle, ...}).
"""
from . import plate_frame
from .motions import Bungeoppang

MENU = ("redbean", "cream", "choco")      # 팥 · 생크림 · 초코
MENU_KO = {"redbean": "팥", "cream": "생크림", "choco": "초코"}


def parse_menu(text: str):
    """CLI 용: 'redbean' → 'redbean',  'redbean,choco' → ['redbean', 'choco']"""
    items = [t.strip() for t in str(text).split(",") if t.strip()]
    return items[0] if len(items) == 1 else items


class MenuBungeoppang(Bungeoppang):
    def menu_for(self, flavor, cells) -> dict:
        """flavor 가 문자열이면 모든 칸이 그 맛, 리스트면 cells 와 같은 순서로 칸별 맛.
        반환: {맛: (칸, ...)} — 처음 나온 순서 유지. 움직이기 전에 검사용으로도 쓴다."""
        cells = tuple(cells)
        flavors = [flavor] * len(cells) if isinstance(flavor, str) else list(flavor)
        if len(flavors) != len(cells):
            raise ValueError(f"칸 {len(cells)}개에 메뉴 {len(flavors)}개 — 개수가 같아야 함")
        materials = self.p.get("materials", {})
        groups: dict = {}
        for c, f in zip(cells, flavors):
            if f not in MENU:
                raise ValueError(f"모르는 메뉴: {f} (가능: {', '.join(MENU)})")
            if f not in materials:
                raise ValueError(f"poses.yaml materials 에 '{f}' 가 없음 (소스통 좌표·짜기 세기 필요)")
            groups.setdefault(f, []).append(c)
        return {f: tuple(cs) for f, cs in groups.items()}

    def add_filling(self, flavor, cells=(0,)):
        groups = self.menu_for(flavor, cells)
        names = ", ".join(f"{MENU_KO[f]}×{len(cs)}" for f, cs in groups.items())
        self.on_step("filling", f"앙금 짜는 중 ({names})")
        for f, cs in groups.items():
            self._dispense(f, cs)
        per_cell = {c: f for f, cs in groups.items() for c in cs}
        self._scene("filling", flavor if isinstance(flavor, str) else per_cell)

    # ---- 철판 뚜껑 틀 (frame.type: plate) — motions.flip_mold 의 흐름은 그대로, 경로만 바꾼다 ----
    def _arc(self, grip, closing: bool):
        f = self.p.get("frame")
        if not plate_frame.is_plate(f):
            return super()._arc(grip, closing)
        n = int(f.get("steps", 8))
        for k in range(1, n + 1):
            deg = 180.0 * (k if closing else n - k) / n
            self.b.move_line(plate_frame.grip_pose(grip, f, deg))

    def flip_mold(self):
        """철판: 뚜껑 닫기 → 앞면 굽기 → (닫힌 묶음째) 뒤집기 → 뒷면 굽기 → 다시 뒤집기 → 뚜껑 열기.
        - 닫기/열기: 뚜껑 판 가장자리를 물고 경첩 둘레로 180° (손목 J6 도 같이 180°)
        - 뒤집기: 닫힌 두 판을 두 판 사이 높이에서 같이 물고, 경첩 둘레로 '위로' 180° 넘겨 로봇 쪽(-x)에 내려놓는다
        - 놓은 뒤에는 옆(+y)으로 retreat_mm[0] 빠져 손가락을 판에서 빼고 위로 retreat_mm[1] 비켜 선다
          (닫힌 자세에서는 팔이 +y 로 멀리 못 빠짐: 에뮬레이터 50mm 이상 NOT REACHABLE)
        - 철판 작업 앞뒤로 홈(관절 이동) → 손목(J4/J6)이 감겨 ±360° 한계에 걸리지 않게 한다"""
        f = self.p.get("frame")
        if not plate_frame.is_plate(f):
            return super().flip_mold()
        from .poses import approach
        grip = self._x("mold_handle")
        side, up = (float(x) for x in f.get("retreat_mm", [30, 80]))
        bake = self.p.get("flip_bake_sec", 3.0)

        def off(q, dy=0.0, dz=0.0):
            q = list(q); q[1] += dy; q[2] += dz
            return q

        def leave(at):                     # 놓고 옆으로 빠졌다가 위로
            self.gripper.release()
            self.b.move_line(off(at, side))
            self.b.move_line(off(at, side, up))

        def take(at):                      # 위 → 옆 → 물기
            self.b.move_line(off(at, side, up))
            self.b.move_line(off(at, side))
            self.b.move_line(at)
            self.gripper.grip()

        self.home()
        # 1) 뚜껑 닫기
        self.on_step("baking", "뚜껑 닫는 중")
        self.gripper.release()
        self.b.move_line(approach(grip, self.dz))
        self.b.move_line(grip)
        self.gripper.grip()
        self._arc(grip, closing=True)
        closed = self._closed_pose()
        leave(closed)
        self._scene("flipped")
        self.on_step("baking", "앞면 굽는 중")
        self.b.wait(bake)
        if f.get("turn", True):
            # 2) 닫힌 묶음째 뒤집기 (+x → 로봇 쪽 -x)
            s0, s1 = plate_frame.stack_pose(grip, f, 0.0), plate_frame.stack_pose(grip, f, 180.0)
            self.on_step("baking", "틀 뒤집는 중")
            take(s0)
            self._turn(grip, forward=True)
            leave(s1)
            self.on_step("baking", "뒷면 굽는 중")
            self.b.wait(bake)
            # 3) 다시 뒤집어 제자리로
            self.on_step("baking", "틀 다시 뒤집는 중")
            take(s1)
            self._turn(grip, forward=False)
            leave(s0)
        # 4) 뚜껑 열기
        self.on_step("baking", "뚜껑 여는 중")
        take(closed)
        self._arc(grip, closing=False)
        self.gripper.release()
        self.b.move_line(approach(grip, self.dz))
        self.home()

    def _turn(self, grip, forward: bool):
        f = self.p["frame"]
        n = int(f.get("steps", 8))
        for k in range(1, n + 1):
            deg = 180.0 * (k if forward else n - k) / n
            self.b.move_line(plate_frame.stack_pose(grip, f, deg))

    def _closed_pose(self):
        f = self.p.get("frame")
        if not plate_frame.is_plate(f):
            return super()._closed_pose()
        return plate_frame.grip_pose(self._x("mold_handle"), f, 180.0)

    def make_batch(self, flavor, n: int = 1):
        if not 1 <= n <= self.n_cells:
            raise ValueError(f"한 번에 1~{self.n_cells}개 (poses.yaml cells.count), 요청 {n}")
        self.menu_for(flavor, range(n))       # 메뉴가 틀리면 로봇이 움직이기 전에 멈춘다
        super().make_batch(flavor, n)         # 반죽 → add_filling(위) → 반죽 덮기 → … (순서는 motions.py 그대로)
