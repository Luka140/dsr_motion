import pyads.errorcodes
import rclpy 
from rclpy.node import Node
from rclpy.impl import rcutils_logger
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_msgs.msg import Header 
from dsr_msgs2.srv.motion import MoveSplineTask




class DSRMotionClient(Node):
    def __init__(self):
        super().__init__("dsr_motion_client")
        NAMESPACE = "/dsr01"

        spline_move_client = self.create_client(MoveSplineTask, f"{NAMESPACE}/motion/move_spline_task", )


        ## SET TCP AND WEIGHT 

            
    def call_splinetask(self):
        ...
        

def main(args=None):
    rclpy.init(args=args)

    motion_client = DSRMotionClient()
    executor = MultiThreadedExecutor()
    
    rclpy.spin(motion_client, executor=executor)
    
    motion_client.destroy_node()
    # Avoid stack trace 
    try:
        rclpy.shutdown()
    except rclpy._rclpy_pybind11.RCLError:
        pass 


if __name__ == '__main__':
    main()
    