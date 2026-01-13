import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseStamped
import sys
import select
import termios
import tty

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        
        # Publishers
        self.pub_a = self.create_publisher(Int32, '/robotA/progress', 10)
        self.pub_b = self.create_publisher(Int32, '/robotB/progress', 10)
        self.pub_target = self.create_publisher(PoseStamped, '/target', 10)
        self.pub_stop = self.create_publisher(Bool, '/stop', 10)
        self.pub_emt = self.create_publisher(Bool, '/emt_arrival_status', 10)
        self.pub_aed = self.create_publisher(Bool, '/robotA/aed_detected', 10)
        self.pub_responder_done = self.create_publisher(Bool, '/robotB/responder_done', 10)

        self.settings = termios.tcgetattr(sys.stdin)
        self.print_usage()

    def print_usage(self):
        print("""
---------------------------------------
Robot Control Node (Keyboard)
---------------------------------------
[Robot A (0~3)]    : 0, 1, 2, 3
[Robot B (0~5)]    : 4, 5, 6, 7, 8, 9 (mapped to 0-5)
[Target Pose]      : 'a' (Target AB), 'b' (Target BA)
[Emergency Stop]   : 's' (STOP), 'f' (FREE)
[EMT Status]       : 'e' (ARRIVED), 'r' (RESET)
[AED Detection]    : 'd' (DETECTED), 'x' (CLEAR)
[Responder Done]   : 'k' (DONE), 'l' (NOT DONE)

CTRL-C to quit
---------------------------------------
""")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def create_pose(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        return msg

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                
                # Robot A: 0 ~ 3
                if key in ['0', '1', '2', '3']:
                    msg = Int32()
                    msg.data = int(key)
                    self.pub_a.publish(msg)
                    print(f"-> /robotA/progress: {msg.data}")

                # Robot B: 4 ~ 9 (mapped to 0 ~ 5)
                elif key in ['4', '5', '6', '7', '8', '9']:
                    msg = Int32()
                    msg.data = int(key) - 4
                    self.pub_b.publish(msg)
                    print(f"-> /robotB/progress: {msg.data}")

                # Target Pose
                elif key == 'a':
                    msg = self.create_pose(0.0, 0.0, 0.0)
                    self.pub_target.publish(msg)
                    print("-> /target: AB (0.0, 0.0, 0.0)")
                elif key == 'b':
                    msg = self.create_pose(1.0, -2.0, 0.0)
                    self.pub_target.publish(msg)
                    print("-> /target: BA (1.0, -2.0, 0.0)")

                # Stop
                elif key == 's':
                    self.pub_stop.publish(Bool(data=True))
                    print("-> /stop: True")
                elif key == 'f':
                    self.pub_stop.publish(Bool(data=False))
                    print("-> /stop: False")

                # EMT Status
                elif key == 'e':
                    self.pub_emt.publish(Bool(data=True))
                    print("-> /emt_arrival_status: True")
                elif key == 'r':
                    self.pub_emt.publish(Bool(data=False))
                    print("-> /emt_arrival_status: False")

                # --- 추가된 코드 부분 ---
                # AED Detection Status
                elif key == 'd':
                    self.pub_aed.publish(Bool(data=True))
                    print("-> /robotA/aed_detected: True")
                elif key == 'x':
                    self.pub_aed.publish(Bool(data=False))
                    print("-> /robotA/aed_detected: False")

                # Responder Done Status
                elif key == 'k':
                    self.pub_responder_done.publish(Bool(data=True))
                    print("-> /robotB/responder_done: True")
                elif key == 'l':
                    self.pub_responder_done.publish(Bool(data=False))
                    print("-> /robotB/responder_done: False")
                # -----------------------

                elif key == '\x03': # CTRL-C
                    break

        except Exception as e:
            print(f"Error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()