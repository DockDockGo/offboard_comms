import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# from example_interfaces.action import YourAction  # Replace with your actual action
import requests
import json
from rclpy.executors import MultiThreadedExecutor

from robot_action_interfaces.action import MissionControl, MissionControlGoal


# MISSION_SERVER = '192.168.0.1:5000' # TODO(sid): Confirm IP address and port
MISSION_SERVER = 'http://192.168.1.2:5000/command'


class AMRTaskClient(Node):

    def __init__(self):
        super().__init__('amr_task_client')
        # self.action_client = ActionClient(self, YourAction, 'amr_action')  # Replace 'YourAction' and 'amr_action' with actual names
        self.timer = self.create_timer(1.0, self.send_get_request,)  # Set the timer interval for sending GET requests
        self.current_task_name = None

    def send_get_request(self):
        try:
            response = requests.get(MISSION_SERVER)  # Replace with your actual URL
            if response.status_code == 200:
                print("status_code = 200")
                print(response)
                data = response.json()
                task_name = data.get('task')
                if (task_name == 'amr_task1' or task_name == 'amr_task2') and data.get('status') == 'START':
                    self.current_task_name = task_name
                    self.get_logger().info(f'Starting task name: {task_name}')
                    self.send_post_request(task_name, 'WIP')
                    # Send AMR from dock id 2 to dock id 1
                    self.send_action(task_name)
                else:
                    self.get_logger().info('No AMR Task found')
                
            else:
                self.get_logger().error(f'GET request failed with status code: {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'An error occurred: {e}')

    def send_post_request(self, task_name, status):
        try:
            requests.post(MISSION_SERVER, json={'name': task_name, 'status': status})  # Replace with your actual URL
        except Exception as e:
            self.get_logger().error(f'An error occurred while sending POST request: {e}')

    def send_action(self, task_name):
        # Make sure to replace 'YourAction' and 'your_goal' with your actual action and goal.
        goal_msg = MissionControl.Goal()
        if task_name == 'amr_task1':
            goal_msg.robot_specific_dock_ids = [1,2,  # AMR1 from dock 1 to 2
                                                2,1]  # AMR2 from dock 2 to 1
        else:
            goal_msg.robot_specific_dock_ids = [2,1,  # AMR1 from dock 1 to 2
                                                1,2]  # AMR1 from dock 2 to 1

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Action completed: {result}')  # Customize the message based on your action's result
        self.send_post_request(self.current_task_name, 'FINISHED')

    def feedback_callback(self, feedback_msg):
        # This is where you can process feedback from the action server if your action provides feedback
        pass


def main(args=None):
    rclpy.init(args=args)
    amr_task_client = AMRTaskClient()
    executor = MultiThreadedExecutor()
    rclpy.spin(amr_task_client, executor=executor)

    amr_task_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
