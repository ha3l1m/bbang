"""RH-P12-RN(-A) 그리퍼를 두산 제어기의 DRL(flange_serial_*)로 직접 제어한다 — 공식 doosan-robot2 만으로 동작, 별도 그리퍼 패키지 불필요.

툴 플랜지 RS-485 로 Modbus RTU 프레임(토크 켜기, Goal Position 0~700)을 보내는 짧은 DRL 프로그램을 만들어
/<ns>/drl/drl_start 로 실행한다 (사용자가 검증한 gripper_drl.py 와 같은 방식).
준비: bringup(mode:=real), 로봇 모드 자동, 펜던트 Tool Flange 출력 24 V.
poses.yaml:  gripper: {type: drl, namespace: /dsr01, move_wait_sec: 1.0}   (가상 모드 시험: virtual: true)
"""
import threading
import time

SLAVE_ID = 1
REG_TORQUE_ENABLE = 256   # 0x0100
REG_GOAL_POSITION = 282   # 0x011A (2 레지스터, 32 bit)


def crc16(data):
    """Modbus RTU CRC16 (다항식 0xA001) → [하위 바이트, 상위 바이트]"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return [crc & 0xFF, crc >> 8]


def frame(data):
    return list(data) + crc16(data)


def fc06_torque_on():
    return frame([SLAVE_ID, 0x06, REG_TORQUE_ENABLE >> 8, REG_TORQUE_ENABLE & 0xFF, 0x00, 0x01])


def fc16_goal_position(pos):
    pos = max(0, min(700, int(pos)))
    return frame([SLAVE_ID, 0x10, REG_GOAL_POSITION >> 8, REG_GOAL_POSITION & 0xFF,
                  0x00, 0x02, 0x04, (pos >> 8) & 0xFF, pos & 0xFF, 0x00, 0x00])


def make_drl(pos, move_wait=1.0):
    """제어기에서 실행될 DRL 코드 (손가락이 움직일 시간 move_wait 초 포함)."""
    return "\n".join([
        "flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)",
        "wait(0.1)",
        f"flange_serial_write(bytes({fc06_torque_on()}))",
        "wait(0.1)",
        "flange_serial_read(0.2)",
        f"flange_serial_write(bytes({fc16_goal_position(pos)}))",
        "wait(0.1)",
        "flange_serial_read(0.2)",
        f"wait({move_wait})",
        "flange_serial_close()",
    ])


class DrlCaller:
    """gripper.py 의 RosTriggerCaller 와 같은 인터페이스(call / set_position)."""

    OPEN_POS, CLOSE_POS = 0, 700

    def __init__(self, namespace="/dsr01", timeout=8.0, move_wait=1.0, virtual=False):
        import rclpy
        from dsr_msgs2.srv import DrlStart, GetDrlState
        from rclpy.executors import SingleThreadedExecutor

        self._DrlStart, self._GetState = DrlStart, GetDrlState
        self.timeout, self.move_wait, self.virtual = timeout, move_wait, virtual
        ns = namespace.rstrip("/") or "/dsr01"
        self.node = rclpy.create_node("boong_drl_gripper", namespace=ns)
        self.start = self.node.create_client(DrlStart, "drl/drl_start")
        self.state = self.node.create_client(GetDrlState, "drl/get_drl_state")
        self._ex = SingleThreadedExecutor()
        self._ex.add_node(self.node)
        threading.Thread(target=self._ex.spin, daemon=True).start()

    def _wait(self, fut, sec):
        done = threading.Event()
        fut.add_done_callback(lambda _f: done.set())
        if not done.wait(sec):
            raise RuntimeError("DRL 서비스 응답 없음")
        return fut.result()

    def set_position(self, pos):
        if not self.start.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError("drl/drl_start 서비스가 안 보임 — bringup(mode:=real) 확인")
        req = self._DrlStart.Request()
        req.robot_system = 1 if self.virtual else 0     # 0 = 실제 로봇, 1 = 가상
        req.code = make_drl(pos, self.move_wait)
        res = self._wait(self.start.call_async(req), self.timeout)
        if not res.success:
            raise RuntimeError("DRL 시작 실패 (로봇 모드 자동? Tool Flange 24 V 출력?)")
        time.sleep(0.3)
        t0 = time.time()
        while time.time() - t0 < self.timeout:          # drl_state 0 = 실행 중 → 끝날 때까지 기다림
            st = self._wait(self.state.call_async(self._GetState.Request()), 2.0)
            if st is not None and st.drl_state != 0:
                return
            time.sleep(0.2)

    def call(self, name):
        self.set_position(self.OPEN_POS if name == "open" else self.CLOSE_POS)
