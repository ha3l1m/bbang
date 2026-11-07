#!/usr/bin/env python3
"""RH-P12-RN-A gripper bridge.

Interface (all relative to the node namespace, e.g. /dsr01):
    gripper/cmd    std_msgs/Float64   0.0 = open ... 1.0 = closed
    gripper/open   std_srvs/Trigger
    gripper/close  std_srvs/Trigger
    gripper/state  std_msgs/Float64   current ratio (published)
    joint_states   sensor_msgs/JointState  finger joints, so RViz shows the motion

Virtual mode : only animates the finger joints (RViz / simulation).
Real mode    : animates the joints AND (optionally) sends Modbus-RTU frames through
               the Doosan controller flange RS-485 using /dsr01/drl/drl_start.
               Real output is OFF by default (real_backend:=off) and, even when on,
               only prints the frames unless dry_run:=false.   See modbus.py: UNVERIFIED.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Int32
from std_srvs.srv import Trigger

from rhp12_gripper import modbus


class GripperNode(Node):
    def __init__(self):
        super().__init__('rhp12_gripper')

        self.declare_parameter('mode', 'virtual')            # virtual | real
        self.declare_parameter('real_backend', 'off')        # off | drl
        self.declare_parameter('dry_run', True)              # True: only log frames
        self.declare_parameter('slave_id', modbus.SLAVE_ID_DEFAULT)
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('pulse_open', modbus.POS_MIN)     # VERIFY which end is open
        self.declare_parameter('pulse_closed', modbus.POS_MAX)
        self.declare_parameter('low_word_first', True)           # VERIFY
        self.declare_parameter('joint_names', ['rhp12_finger_l_joint', 'rhp12_finger_r_joint'])
        self.declare_parameter('open_per_finger', 0.053)     # [m] 106 mm stroke / 2
        # joint value at open / closed. Default = old prismatic model.
        # Official rh_p12_rn_a_description: joint_names=['rh_r1'], joint_open=0.0, joint_closed=1.1
        self.declare_parameter('joint_open', 0.053)
        self.declare_parameter('joint_closed', 0.0)
        self.declare_parameter('speed', 1.0)                 # [ratio/s]
        self.declare_parameter('publish_rate', 30.0)

        p = self.get_parameter
        self.mode = p('mode').value
        self.real_backend = p('real_backend').value
        self.dry_run = p('dry_run').value
        self.slave = int(p('slave_id').value)
        self.baud = int(p('baudrate').value)
        self.pulse_open = int(p('pulse_open').value)
        self.pulse_closed = int(p('pulse_closed').value)
        self.low_word_first = p('low_word_first').value
        self.joint_names = list(p('joint_names').value)
        self.open_per_finger = float(p('open_per_finger').value)
        self.joint_open = float(p('joint_open').value)
        self.joint_closed = float(p('joint_closed').value)
        self.speed = float(p('speed').value)
        rate = float(p('publish_rate').value)

        self.current = 0.0   # ratio, start open
        self.target = 0.0
        self.torque_sent = False

        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.state_pub = self.create_publisher(Float64, 'gripper/state', 10)
        self.create_subscription(Float64, 'gripper/cmd', self._on_cmd, 10)
        # boong_robot (ros_service gripper) sends finger position 0 (open) .. 700 (closed)
        self.create_subscription(Int32, 'gripper/position_cmd',
                                 lambda m: self._set_target(m.data / 700.0), 10)
        self.create_service(Trigger, 'gripper/open', lambda rq, rs: self._srv(rq, rs, 0.0))
        self.create_service(Trigger, 'gripper/close', lambda rq, rs: self._srv(rq, rs, 1.0))

        self.drl_client = None
        if self.mode == 'real' and self.real_backend == 'drl':
            try:
                from dsr_msgs2.srv import DrlStart
                self._DrlStart = DrlStart
                self.drl_client = self.create_client(DrlStart, 'drl/drl_start')
            except ImportError:
                self.get_logger().error('dsr_msgs2 not found: real output disabled')

        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'RH-P12-RN-A bridge: mode={self.mode} real_backend={self.real_backend} '
            f'dry_run={self.dry_run}')
        if self.mode == 'real' and self.real_backend != 'drl':
            self.get_logger().warn('real mode but real_backend=off: RViz animation only, '
                                   'the physical gripper will NOT move')

    # ---- commands -------------------------------------------------------
    def _set_target(self, ratio: float):
        self.target = max(0.0, min(1.0, float(ratio)))
        self._send_real(self.target)

    def _on_cmd(self, msg: Float64):
        self._set_target(msg.data)

    def _srv(self, _req, res, ratio):
        self._set_target(ratio)
        res.success = True
        res.message = f'target={ratio:.2f}'
        return res

    # ---- real output ----------------------------------------------------
    def _send_real(self, ratio: float):
        if self.mode != 'real' or self.real_backend != 'drl':
            return
        frames = []
        if not self.torque_sent:
            frames.append(modbus.torque_enable_frame(self.slave, True))
        pulse = modbus.ratio_to_pulse(ratio, self.pulse_open, self.pulse_closed)
        frames.append(modbus.goal_position_frame(self.slave, pulse, self.low_word_first))
        code = modbus.build_drl(frames, baudrate=self.baud)

        if self.dry_run:
            hexes = ' | '.join(f.hex(' ') for f in frames)
            self.get_logger().info(f'[dry_run] pulse={pulse} frames: {hexes}')
            return
        if self.drl_client is None or not self.drl_client.service_is_ready():
            self.get_logger().error('drl/drl_start not available: command not sent')
            return
        req = self._DrlStart.Request()
        req.robot_system = 0  # 0: real
        req.code = code
        self.drl_client.call_async(req)
        self.torque_sent = True
        self.get_logger().info(f'sent DRL (pulse={pulse})')

    # ---- animation + joint states --------------------------------------
    def _tick(self):
        step = self.speed * self.dt
        if abs(self.target - self.current) <= step:
            self.current = self.target
        else:
            self.current += step if self.target > self.current else -step

        q = self.joint_open + (self.joint_closed - self.joint_open) * self.current
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = [q] * len(self.joint_names)
        self.js_pub.publish(js)
        self.state_pub.publish(Float64(data=self.current))


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
