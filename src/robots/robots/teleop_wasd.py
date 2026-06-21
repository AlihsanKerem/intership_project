import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
        w
   a    s    d

space key, x : force stop

CTRL-C to quit
"""

moveBindings = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopWASD(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.5
        self.turn = 1.0

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        print(msg)
        try:
            while rclpy.ok():
                key = getKey(settings)
                
                twist = Twist()
                if key in moveBindings.keys():
                    twist.linear.x = moveBindings[key][0] * self.speed
                    twist.angular.z = moveBindings[key][1] * self.turn
                elif key == ' ' or key == 'x':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == '\x03': # CTRL-C
                    break
                else:
                    # Do nothing if no key is pressed
                    continue
                
                self.publisher_.publish(twist)
                    
        except Exception as e:
            self.get_logger().error(f"Error reading keyboard: {e}")
        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    teleop = TeleopWASD()
    teleop.run()
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
