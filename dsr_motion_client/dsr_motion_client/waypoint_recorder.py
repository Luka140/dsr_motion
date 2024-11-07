import rclpy 
from rclpy.node import Node
from rclpy.impl import rcutils_logger
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import Header 
from std_srvs.srv import Trigger 
from dsr_msgs2.srv.motion import MoveSplineTask
from dsr_msgs2.srv.aux_control import GetCurrentPosj, GetCurrentPosx

import copy 
from functools import partial 
from datetime import datetime
import numpy as np
import pathlib 


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__("dsr_waypoint_recorder")
        self.NAMESPACE = "/dsr01"

        reentrent_callback = ReentrantCallbackGroup()
        self.posj_client = self.create_client(GetCurrentPosj, f"{self.NAMESPACE}/aux_control/get_current_posj", callback_group=reentrent_callback)
        self.posx_client = self.create_client(GetCurrentPosx, f"{self.NAMESPACE}/aux_control/get_current_posx", callback_group=reentrent_callback)

        exclusive_callback = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, '~/register_position', self.register_position, callback_group=exclusive_callback)
        self.create_service(Trigger, '~/write_to_file', self.write_to_file, callback_group=exclusive_callback)

        self.positions = []

        self.ref_frame_posx = 0  # Reference frame 0 == DR_BASE 
        self.default_velocity = 0
        self.default_acceleration = 0
        self.default_duration = 0

        self.received_posj, self.received_posjx = False 

        self.filepath = pathlib.Path.cwd() / 'src' / 'dsr_motion' / 'trajectories'
    

    def register_position(self, request, response):
        self.received_posx, self.received_posj = False, False 
        position_dict = {}
        #call posj and posx
        posj_call = self.posj_client.call_async(GetCurrentPosj())
        posj_call.add_done_callback(partial(self.posj_received, position_dict))
        posx_call = self.posx_client.call_async(GetCurrentPosx(ref=self.ref_frame_posx))
        posx_call.add_done_callback(partial(self.posx_received, position_dict))

        check_rate = self.create_rate(5)
        while not self.received_posj and self.received_posx:
            check_rate.sleep()
        
        self.positions.append(position_dict)

        response.success = True 
        return response


    def posj_received(self, future, position_dict):
        result = future.result()
        if not result.success:
            raise ...

        position_dict["posj"] = future.task_pos_info[:5]
        self.received_posj = True

    def posx_received(self, future, position_dict):
        result = future.result()
        if not result.success:
            raise ...

        position_dict["posx"] = future.task_pos_info[:5]
        self.received_posjx = True

    def write_to_file(self, request, response):
        # Write positions to a waypoint file 
        positions = copy.deepcopy(self.positions)
        self.positions = []
        
        filename = f'{self.filepath}/trajectory_{datetime.now()}.yaml'
        self.get_logger().info(f'Creating config file at {filename}')

        indent = '  '
        with open(filename, 'w') as f:
            # Header
            # f.write('publisher_scaled_joint_trajectory_controller:\n')
            f.write('/**:\n')
            f.write(f'{indent}ros__parameters:\n\n')
            f.write(f'{indent*2}controller_name: "{self.NAMESPACE}"\n\n')

            # Waypoints
            nr_of_waypoints = len(positions)
            f.write(f'{indent*2}goal_names: {[f"waypoint{i}" for i in range(nr_of_waypoints)]}\n')

            for i in range(nr_of_waypoints):
                f.write(f'{indent*2}waypoint{i}:\n')
                f.write(f'{indent*3}posj: {np.array2string(positions[i]["posj"], precision=10, separator=",", max_line_width=10**3)}\n')
                f.write(f'{indent*3}posx:')
                f.write(f'{indent*4}pos: {np.array2string(positions[i]["posx"], precision=10, separator=",", max_line_width=10**3)}\n')
                f.write(f'{indent*4}reference_enum: {self.ref_frame_posx}\n')

                f.write(f'{indent*3}velocity: {self.default_velocity}\n')
                f.write(f'{indent*3}acceleration: {self.default_acceleration}\n')

                # Movement duration
                f.write(f'{indent*3}movement_duration: [{int(self.default_movement_duration)}, {int((self.default_movement_duration - int(self.default_movement_duration))/10**9)}]\n')

                # grinder turned off by default
                f.write(f'{indent*3}grinder_on: false\n\n')

        self.get_logger().info(f'Created {filename} with {nr_of_waypoints} trajectory points.\n Resetting trajectory list...')

        response.success = True 
        return response        

def main(args=None):
    rclpy.init(args=args)

    waypoint_recorder = WaypointRecorder()
    executor = MultiThreadedExecutor()
    
    rclpy.spin(waypoint_recorder, executor=executor)
    
    waypoint_recorder.destroy_node()

    try:
        rclpy.shutdown()
    except rclpy._rclpy_pybind11.RCLError:
        pass 


if __name__ == '__main__':
    main()
    