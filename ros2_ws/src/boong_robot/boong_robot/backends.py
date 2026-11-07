"""로봇 제어 백엔드.
- DsrBackend: 두산 DSR_ROBOT2 (virtual 에뮬레이터 / real 로봇 동일)
- MockBackend: 로봇 없이 로그+대기만 (웹 개발, 단위 테스트용)
motions.py는 이 인터페이스만 사용한다: home / move_line / rotate_j6 / set_do / wait
"""
import time


class MockBackend:
    def __init__(self, log=print, speed: float = 0.2):
        self.log = log
        self.speed = speed  # 동작 1번당 대기 (초)
        self.calls: list[tuple] = []

    def _do(self, *call):
        self.calls.append(call)
        self.log(f"[mock] {call}")
        if self.speed:
            time.sleep(self.speed)

    def home(self, posj):
        self._do("home", tuple(posj))

    def move_line(self, posx):
        self._do("movel", tuple(round(v, 1) for v in posx))

    def rotate_j6(self, deg):
        self._do("rotate_j6", deg)

    def set_do(self, pin, value):
        self._do("set_do", pin, value)

    def wait(self, sec):
        self.calls.append(("wait", sec))
        time.sleep(min(sec, self.speed) if self.speed else 0)


class DsrBackend:
    """DR_init에 노드를 등록한 뒤에 생성해야 함 (robot_action_server.py 참고)."""

    def __init__(self, speed: dict, log=print, tool: dict | None = None):
        from DSR_ROBOT2 import (  # noqa: import 시점에 DR_init 설정이 필요
            get_current_posj, movej, movel, posj, posx,
            set_digital_output, set_velj, set_accj, set_velx, set_accx,
        )
        self._movej, self._movel = movej, movel
        self._posj, self._posx = posj, posx
        self._cur_j = get_current_posj
        self._set_do = set_digital_output
        self.vj, self.aj = speed.get("vel_j", 30), speed.get("acc_j", 30)
        self.vx, self.ax = speed.get("vel_x", 100), speed.get("acc_x", 100)
        set_velj(self.vj); set_accj(self.aj)
        set_velx(self.vx, 30); set_accx(self.ax, 60)
        self.log = log
        if tool:   # 펜던트에 미리 등록해 둔 툴(그리퍼+통 무게)/TCP 이름을 선택 — 무게·TCP 가 틀리면 충돌 감지·정확도가 나빠진다
            from DSR_ROBOT2 import set_tcp, set_tool
            if tool.get("name"):
                set_tool(tool["name"]); log(f"툴 설정: {tool['name']}")
            if tool.get("tcp"):
                set_tcp(tool["tcp"]); log(f"TCP 설정: {tool['tcp']}")

    def home(self, posj):
        self._movej(self._posj(*posj), vel=self.vj, acc=self.aj)

    def move_line(self, posx):
        self._movel(self._posx(*posx), vel=self.vx, acc=self.ax)

    def rotate_j6(self, deg):
        j = list(self._cur_j())
        j[5] += deg
        self._movej(self._posj(*j), vel=self.vj, acc=self.aj)

    def set_do(self, pin, value):
        self._set_do(pin, value)

    def wait(self, sec):
        time.sleep(sec)
