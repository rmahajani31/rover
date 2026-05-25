from pathlib import Path
import sys
import types
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

if 'rclpy' not in sys.modules:
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *args, **kwargs: None
    rclpy.shutdown = lambda: None
    sys.modules['rclpy'] = rclpy

    rclpy_executors = types.ModuleType('rclpy.executors')
    rclpy_executors.ExternalShutdownException = type('ExternalShutdownException', (Exception,), {})
    sys.modules['rclpy.executors'] = rclpy_executors

    class DummyNode:
        def __init__(self, *args, **kwargs):
            pass

    rclpy_node = types.ModuleType('rclpy.node')
    rclpy_node.Node = DummyNode
    sys.modules['rclpy.node'] = rclpy_node

    class DummyQoSProfile:
        def __init__(self, *args, **kwargs):
            pass

    rclpy_qos = types.ModuleType('rclpy.qos')
    rclpy_qos.QoSProfile = DummyQoSProfile
    rclpy_qos.ReliabilityPolicy = type('ReliabilityPolicy', (), {'RELIABLE': object()})
    rclpy_qos.DurabilityPolicy = type('DurabilityPolicy', (), {'TRANSIENT_LOCAL': object()})
    rclpy_qos.HistoryPolicy = type('HistoryPolicy', (), {'KEEP_LAST': object()})
    sys.modules['rclpy.qos'] = rclpy_qos

if 'sensor_msgs.msg' not in sys.modules:
    sensor_msgs = types.ModuleType('sensor_msgs')
    sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')

    class Joy:
        def __init__(self):
            self.axes = []
            self.buttons = []

    sensor_msgs_msg.Joy = Joy
    sys.modules['sensor_msgs'] = sensor_msgs
    sys.modules['sensor_msgs.msg'] = sensor_msgs_msg

if 'geometry_msgs.msg' not in sys.modules:
    geometry_msgs = types.ModuleType('geometry_msgs')
    geometry_msgs_msg = types.ModuleType('geometry_msgs.msg')

    class TwistStamped:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None)
            self.twist = SimpleNamespace(
                linear=SimpleNamespace(x=0.0),
                angular=SimpleNamespace(z=0.0),
            )

    geometry_msgs_msg.TwistStamped = TwistStamped
    sys.modules['geometry_msgs'] = geometry_msgs
    sys.modules['geometry_msgs.msg'] = geometry_msgs_msg

if 'std_msgs.msg' not in sys.modules:
    std_msgs = types.ModuleType('std_msgs')
    std_msgs_msg = types.ModuleType('std_msgs.msg')

    class Bool:
        def __init__(self, data=False):
            self.data = data

    std_msgs_msg.Bool = Bool
    sys.modules['std_msgs'] = std_msgs
    sys.modules['std_msgs.msg'] = std_msgs_msg

from gamepad_adapter.adapter_node import AdapterNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeLogger:
    def __init__(self):
        self.records = []

    def info(self, msg):
        self.records.append(('info', msg))

    def warn(self, msg):
        self.records.append(('warn', msg))


def make_adapter(estop_state=False):
    node = AdapterNode.__new__(AdapterNode)
    node.pub_estop = FakePublisher()
    node.pub_cmd = FakePublisher()
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: SimpleNamespace())
    )

    node.estop_state = estop_state
    node.desired_linear_x = 1.0
    node.desired_angular_z = -1.0
    node.current_linear_x = 0.5
    node.current_angular_z = -0.5

    node.throttle_axis = 1
    node.steer_axis = 3
    node.deadzone = 0.06
    node.max_linear_x = 0.35
    node.max_angular_z = 0.8
    node.throttle_expo = 0.5
    node.steer_expo = 0.6
    node.publish_rate_hz = 30.0
    node.accel_rate_linear = 0.6
    node.decel_rate_linear = 1.5
    node.accel_rate_angular = 1.2
    node.decel_rate_angular = 2.5
    node.invert_throttle = False
    node.invert_steer = False
    node.swapped = False
    node.btn_estop = [0]
    node.btn_swap = -1
    node.btn_inv_throttle = -1
    node.btn_inv_steer = -1
    node.prev_buttons = None
    return node


def test_set_estop_state_publishes_latched_true_and_zeros_commands():
    node = make_adapter(estop_state=False)

    node._set_estop_state(True, 'startup active', log_level='warn')

    assert node.estop_state is True
    assert node.desired_linear_x == 0.0
    assert node.desired_angular_z == 0.0
    assert node.current_linear_x == 0.0
    assert node.current_angular_z == 0.0
    assert node.pub_estop.messages[-1].data is True
    assert node.pub_cmd.messages[-1].twist.linear.x == 0.0
    assert node.pub_cmd.messages[-1].twist.angular.z == 0.0


def test_on_joy_ignores_axes_while_estop_active():
    node = make_adapter(estop_state=True)
    msg = SimpleNamespace(axes=[0.0, 1.0, 0.0, -1.0], buttons=[0, 0, 0, 0])

    node.on_joy(msg)

    assert node.desired_linear_x == 0.0
    assert node.desired_angular_z == 0.0
    assert node.prev_buttons == [0, 0, 0, 0]


def test_on_joy_estop_button_clears_startup_latch():
    node = make_adapter(estop_state=True)
    node.prev_buttons = [0]
    msg = SimpleNamespace(axes=[0.0, 0.0, 0.0, 0.0], buttons=[1])

    node.on_joy(msg)

    assert node.estop_state is False
    assert node.pub_estop.messages[-1].data is False
    assert node.pub_cmd.messages[-1].twist.linear.x == 0.0
    assert node.pub_cmd.messages[-1].twist.angular.z == 0.0
    assert node.prev_buttons == [1]
