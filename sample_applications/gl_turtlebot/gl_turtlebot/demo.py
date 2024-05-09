import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nav2_msgs.action import NavigateToPose

import time

class TurtlebotDemo(Node):
    def __init__(self):
        super().__init__('turtlebot_demo')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def navigate(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        goal_msg.pose.header.frame_id = 'map'

        self.client.wait_for_server()
        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        return send_goal_future
    
    def grab_frame(self, camera_topic: str):
        print(f'Grabbing frame for {camera_topic}...')
    
    def submit_image_query(self):
        print('Submitting image query...')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Navigation goal rejected')
            return

        self.get_logger().info('Navigation goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Navigation goal reached')
        else:
            self.get_logger().info('Navigation goal failed')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotDemo()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    goals = [
        (-3.800978080802737, -2.7160307501669427, 0.0, 0.0, 0.0, -0.7458165911298489, 0.6661513434614178),
        (-14.006568822464489, -2.510155155879514, 0.0, 0.0, 0.0, -0.5383401759351223, 0.8427276279879173),
        (-13.20998832378874, -23.924445731658412, 0.0, 0.0, 0.0, 0.1576575814757942, 0.9874938415014057),
        (-0.6006570063646215, -22.95755948389724, 0.0, 0.0, 0.0, 0.4988048549377298, 0.8667143224214945),
        (4.599254944774189, -24.610295710617095, 0.0, 0.0, 0.0, -0.2715895473737527, 0.9624131741395273),
        (7.501646099704738, -23.007248658524453, 0.0, 0.0, 0.0, 0.49791322777441605, 0.8672268547544307),
        (9.89902232082387, -3.000549456119071, 0.0, 0.0, 0.0, 0.9519339188635288, 0.30630346736059716),
    ]
    laps = 0
    while True:
        t1 = time.time()
        for goal in goals:
            future = node.navigate(*goal)
            rclpy.spin_until_future_complete(node, future)

            # Handling results within the loop after each spin
            if future.done():
                goal_handle = future.result()
                if goal_handle.accepted:
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(node, result_future)
                    result = result_future.result().result
                    if result:
                        node.get_logger().info(f'Goal {goal} reached.')
                    else:
                        node.get_logger().info(f'Failed to reach goal {goal}.')
                else:
                    node.get_logger().info(f'Goal {goal} was rejected.')
        
        laps += 1
        node.get_logger().info(f'Completed lap in {time.time() - t1:.2f}s. Total laps: {laps}')
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
