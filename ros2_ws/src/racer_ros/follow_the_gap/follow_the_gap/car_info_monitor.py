import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys

class Monitor(Node):
    def __init__(self):
        super().__init__('monitor')
        self.sub = self.create_subscription(Float32MultiArray, '/car_info_debug', self.debug_callback, 10)

        sys.stdout.write(f'DEBUGGER INITIALIZED...\n')
        
    def debug_callback(self, msg):
        speed, steering, gap_start, gap_end, best = msg.data
        gap_width = int(gap_end - gap_start + 1)
        
        line = (
            f'\r\033[K'
            f'SPEED: {speed:5.2f} | '
            f'STEER: {steering:+6.2f} | '
            f'GAP: [{int(gap_start):4d}, {int(gap_end):4d}] (w={gap_width:4d}) | '
            f'BEST: {int(best):4d} | '
        )
        
        sys.stdout.write(f'\r{line}')
        sys.stdout.flush()
        
def main(args=None):
    rclpy.init(args=args)
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        sys.stdout.write('\n')  # newline on exit so prompt isn't on the dashboard line
        sys.stdout.flush()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()