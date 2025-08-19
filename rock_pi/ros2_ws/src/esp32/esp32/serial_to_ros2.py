#!/usr/bin/env python3
import rclpy, serial, time, re
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

FLOAT_RE = r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?'

class Bridge(Node):
    def __init__(self):
        super().__init__('esp32_imu_serial_bridge')
        self.declare_parameter('port', '/dev/ttyESP32')
        self.declare_parameter('baud', 115200)
        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)

        self.ser = None
        self._connect_serial()

        self.roll_pub  = self.create_publisher(Float32, 'roll', 10)
        self.pitch_pub = self.create_publisher(Float32, 'pitch', 10)
        self.yaw_pub   = self.create_publisher(Float32, 'yaw', 10)
        self.rpy_pub   = self.create_publisher(Vector3, 'rpy', 10)  # x=roll, y=pitch, z=yaw
        self.timer = self.create_timer(0.005, self._tick)           # ~200 Hz

        self.msg_count = 0
        self.last_log  = time.time()

    def _connect_serial(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f'Opening {self.port} @ {self.baud}...')
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                time.sleep(0.5)
                self.ser.reset_input_buffer()
                self.get_logger().info('Serial connected.')
                return
            except Exception as e:
                self.get_logger().warn(f'Failed to open serial: {e}. Retrying in 1s...')
                time.sleep(1.0)

    def _tick(self):
        if self.ser is None:
            self._connect_serial()
            return
        try:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                return

            vals = self._parse_line(line)
            if not vals:
                return

            # Publish what we have
            roll  = vals.get('roll')
            pitch = vals.get('pitch')
            yaw   = vals.get('yaw')

            if roll is not None:
                self.roll_pub.publish(Float32(data=roll))
            if pitch is not None:
                self.pitch_pub.publish(Float32(data=pitch))
            if yaw is not None:
                self.yaw_pub.publish(Float32(data=yaw))

            # Always publish rpy vector; fill missing fields with 0.0
            v = Vector3()
            v.x = roll  if roll  is not None else 0.0
            v.y = pitch if pitch is not None else 0.0
            v.z = yaw   if yaw   is not None else 0.0
            self.rpy_pub.publish(v)

            # heartbeat
            self.msg_count += 1
            now = time.time()
            if now - self.last_log > 2.0:
                hz = self.msg_count / max(1e-3, (now - self.last_log))
                self.get_logger().info(f'Publishing @ ~{hz:.1f} Hz')
                self.msg_count = 0
                self.last_log  = now

        except serial.SerialException as e:
            self.get_logger().warn(f'Serial error: {e}. Reconnecting...')
            try: self.ser.close()
            except Exception: pass
            self.ser = None
            time.sleep(0.5)
        except Exception:
            # ignore malformed lines
            pass

    def _parse_line(self, s: str):
        # Exact prefixes to avoid "$RRP" noise:
        if s.startswith('$RP,'):
            # $RP,<roll_deg>,<pitch_deg>
            parts = s.split(',', 2)
            if len(parts) >= 3:
                return {'roll': float(parts[1]), 'pitch': float(parts[2])}

        if s.startswith('$RY,'):
            # $RY,<roll_deg>,<yaw_deg>
            parts = s.split(',', 2)
            if len(parts) >= 3:
                return {'roll': float(parts[1]), 'yaw': float(parts[2])}

        # Named tokens: roll=.. pitch=.. yaw=..
        vals = {}
        for name in ('roll','pitch','yaw'):
            m = re.search(fr'{name}\s*=\s*({FLOAT_RE})', s, re.IGNORECASE)
            if m: vals[name] = float(m.group(1))
        if vals:
            return vals

        # Fallback: first two floats → roll, second → yaw (if the line uses RY),
        # otherwise → pitch (if the line uses RP). If neither, assume roll,yaw.
        floats = re.findall(FLOAT_RE, s)
        if len(floats) >= 2:
            f1, f2 = float(floats[0]), float(floats[1])
            if 'RP' in s:
                return {'roll': f1, 'pitch': f2}
            elif 'RY' in s:
                return {'roll': f1, 'yaw': f2}
            else:
                return {'roll': f1, 'yaw': f2}
        return None

def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        if node.ser:
            try: node.ser.close()
            except Exception: pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
