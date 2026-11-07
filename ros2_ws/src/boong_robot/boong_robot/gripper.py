"""그리퍼 여닫기. type 으로 방식을 고른다 (poses.yaml 의 gripper 항목).

  mock          로봇 없이 기다리기만 함 (에뮬레이터/테스트)
  ros_service   e0509_gripper_description 패키지(RH-P12-RN-A용, Humble 포크)의 — 우리 그리퍼는 RN 이라 호환 미확인, 실로봇은 drl 권장
                Trigger 서비스 <namespace>/open, <namespace>/close 를 호출   ← 실제 로봇용
  digital_out   컨트롤러 디지털 출력으로 여닫기 (핀 번호 필요)
  drl           공식 doosan-robot2 만으로: DRL flange_serial 로 RH-P12-RN(-A) 에 Modbus 전송 (drl_gripper.py)  ← 그리퍼 패키지 없이 실로봇

소스통을 짜는 동작은 손가락 위치(0 열림 ~ 700 닫힘)를 직접 지정한다(hold/squeeze).
ros_service 는 <namespace>/position_cmd (Int32) 토픽으로 보낸다. mock/digital_out 은 기다리기만 한다.

ros_service 는 포크한 doosan-robot2(fhekwn549/doosan-robot2, humble)와
그리퍼 패키지의 bringup(mode:=real)이 떠 있어야 서비스가 보인다.
"""
import threading

_CALLERS: dict = {}


class RosTriggerCaller:
    """Trigger 서비스를 전용 노드(백그라운드 스레드)로 호출한다.
    action 콜백 안에서 불러도 같은 콜백 그룹을 막지 않아 교착이 안 난다."""

    def __init__(self, namespace: str, timeout: float = 5.0):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_srvs.srv import Trigger

        self._Trigger = Trigger
        self.ns = namespace.rstrip("/")
        self.timeout = timeout
        self.node = rclpy.create_node("boong_gripper_client")
        from std_msgs.msg import Int32

        self._Int32 = Int32
        self.clients = {n: self.node.create_client(Trigger, f"{self.ns}/{n}") for n in ("open", "close")}
        self.pos_pub = self.node.create_publisher(Int32, f"{self.ns}/position_cmd", 10)
        self._ex = SingleThreadedExecutor()
        self._ex.add_node(self.node)
        threading.Thread(target=self._ex.spin, daemon=True).start()

    def set_position(self, pos: int) -> None:
        self.pos_pub.publish(self._Int32(data=int(pos)))

    def call(self, name: str) -> None:
        cli = self.clients[name]
        if not cli.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError(f"그리퍼 서비스 {self.ns}/{name} 가 안 보임 — 그리퍼 bringup(mode:=real) 실행 확인")
        done = threading.Event()
        fut = cli.call_async(self._Trigger.Request())
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(self.timeout):
            raise RuntimeError(f"그리퍼 {name} 응답 없음 ({self.timeout}s)")
        res = fut.result()
        if not res.success:
            raise RuntimeError(f"그리퍼 {name} 실패: {res.message}")


def _drl_caller_for(cfg: dict):
    from .drl_gripper import DrlCaller

    key = ("drl", cfg.get("namespace", "/dsr01"))
    if key not in _CALLERS:
        _CALLERS[key] = DrlCaller(key[1], cfg.get("timeout_sec", 8.0), cfg.get("move_wait_sec", 1.0), cfg.get("virtual", False))
    return _CALLERS[key]


def _caller_for(cfg: dict) -> RosTriggerCaller:
    ns = cfg.get("namespace", "/dsr01/gripper")
    if ns not in _CALLERS:  # 붕어빵마다 노드를 새로 만들지 않도록 재사용
        _CALLERS[ns] = RosTriggerCaller(ns, cfg.get("timeout_sec", 5.0))
    return _CALLERS[ns]


class Gripper:
    def __init__(self, backend, cfg: dict, caller=None):
        self.b = backend
        self.type = cfg.get("type", "mock")
        self.wait_sec = cfg.get("wait_sec", 0.8)
        self.close_seq = cfg.get("close", [])
        self.open_seq = cfg.get("open", [])
        self.caller = caller
        if self.type == "ros_service" and self.caller is None:
            self.caller = _caller_for(cfg)
        if self.type == "drl" and self.caller is None:
            self.caller = _drl_caller_for(cfg)

    def _apply(self, name: str, seq):
        if self.type == "digital_out":
            for pin, value in seq:
                self.b.set_do(pin, value)
        elif self.type in ("ros_service", "drl"):
            self.caller.call(name)
        self.b.wait(self.wait_sec)

    def _position(self, pos: int):
        if self.type in ("ros_service", "drl"):
            self.caller.set_position(pos)

    def hold(self, pos: int):
        """통을 가볍게 잡기 (손가락이 닿는 정도)."""
        self._position(pos)
        self.b.wait(self.wait_sec)

    def squeeze(self, pos: int, sec: float, back_to: int):
        """pos까지 더 닫아서 sec초 누르고, back_to(잡기 위치)로 되돌린다."""
        self._position(pos)
        self.b.wait(sec)
        self._position(back_to)
        self.b.wait(self.wait_sec)

    def grip(self):
        self._apply("close", self.close_seq)

    def release(self):
        self._apply("open", self.open_seq)
