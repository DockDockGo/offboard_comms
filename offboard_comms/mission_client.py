import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import requests
import json
from datetime import datetime
from rclpy.executors import MultiThreadedExecutor
from enum import Enum

from robot_action_interfaces.action import MissionControl
from std_msgs.msg import String
from .testbed_config import TaskStatus, AMR, WorkCell

SENTINEL_DOCK_ID = 100
CANCEL_SENTINEL_VALUE = 0

TESTBED_EMULATOR_APP_SERVER_IP = 'http://192.168.1.9:8000'  # Point the IP to the fleet infra backend server
POLLING_PERIOD_S = 1.0  # How often should this client poll fleet infra to fetch newly enqueued missions

class AMRTaskClient(Node):

    def __init__(self):
        super().__init__('amr_task_client')
        self.action_client = ActionClient(self, MissionControl, '/MissionControl')  # Replace 'YourAction' and 'amr_action' with actual names
        self.timer = self.create_timer(1.0, self.start_enqueued_missions)  # Set the timer interval for sending GET requests
        self.amr_mission_urls = {
            1: None,
            2: None,
        }
        self.amr_mission_goals = {
            1: None,
            2: None,
        }

    def start_enqueued_missions(self):
        self.get_logger().info("\n")

        self.get_logger().info(f"{self.amr_mission_urls=}")
        self.get_logger().info(f"{self.amr_mission_goals=}")

        # GET request to testbed emulator server to fetch tasks
        amr_1_mission = self.get_enqueued_amr_missions(amr_id=AMR.AMR_1)
        amr_2_mission = self.get_enqueued_amr_missions(amr_id=AMR.AMR_2)

        
        missions_to_launch_together = []
        if amr_1_mission is not None:
            missions_to_launch_together.append(amr_1_mission)
        if amr_2_mission is not None:
            missions_to_launch_together.append(amr_2_mission)
        
        self.get_logger().info(f'missions_to_launch_together: {missions_to_launch_together}')

        # Dictionary storing whether the undocking stage should be skipped for each AMR
        amr_execute_undocking = {
            1: 1,
            2: 1,
        }
        self.get_logger().info(f"{amr_execute_undocking=}")
        # Mark any missions being over-ridden as completed, and skip undocking stage for such AMRs
        for mission in missions_to_launch_together:
            already_running_mission_url = self.amr_mission_urls[mission['amr_id']]
            if already_running_mission_url is not None:
                self.get_logger().info(f"Marking mission being over-ridden as COMPLETED: {already_running_mission_url}")
                self.update_amr_mission_status(already_running_mission_url, TaskStatus.COMPLETED)
                amr_execute_undocking[mission['amr_id']] = 0

        # Mark latest missions received as RUNNING and update in-file fleet state
        for mission in missions_to_launch_together:
            self.update_amr_mission_status(mission['url'], TaskStatus.RUNNING)
            self.amr_mission_urls[mission['amr_id']] = mission['url']
            self.amr_mission_goals[mission['amr_id']] = mission['goal']

        if len(missions_to_launch_together) == 1:
            mission = missions_to_launch_together[0]
            mission_amr_id = mission["amr_id"]
            other_amr_id = 1 if mission_amr_id==2 else 2
            self.get_logger().info(f'Sending AMR: {mission_amr_id} from {mission["start"]} to {mission["goal"]}')
            # Fetch existing goal for other AMR, if any
            existing_other_amr_goal = self.amr_mission_goals[other_amr_id]
            other_amr_goal = existing_other_amr_goal if existing_other_amr_goal is not None else SENTINEL_DOCK_ID
            if mission_amr_id == 1:
                # action_goal_params = [mission['start'],
                #                       mission['goal'],
                #                       SENTINEL_DOCK_ID,
                #                       other_amr_goal]
                action_goal_params = [SENTINEL_DOCK_ID,
                                      mission['goal'],
                                      SENTINEL_DOCK_ID,
                                      other_amr_goal]
            elif mission_amr_id == 2:
                # action_goal_params = [SENTINEL_DOCK_ID,
                #                       other_amr_goal,
                #                       mission['start'], 
                #                       mission['goal']]
                action_goal_params = [SENTINEL_DOCK_ID,
                                      other_amr_goal,
                                      SENTINEL_DOCK_ID, 
                                      mission['goal']]
            # First cancel any ongoing missions
            self.cancel_ongoing_amr_action_servers()
            self.send_mission_control_action(action_goal_params, list(amr_execute_undocking.values()))
        elif len(missions_to_launch_together) == 2:
            self.get_logger().info(f'Sending AMR_1 from: {amr_1_mission["start"]} to {amr_1_mission["goal"]}, and AMR_2 from: {amr_2_mission["start"]} to {amr_2_mission["goal"]}')
            action_goal_params = [amr_1_mission['start'],
                                  amr_1_mission['goal'],
                                  amr_2_mission['start'],
                                  amr_2_mission['goal']]
            self.cancel_ongoing_amr_action_servers()
            self.send_mission_control_action(action_goal_params, list(amr_execute_undocking.values()))
        

    def get_enqueued_amr_missions(self, amr_id=None, only_fetch_first_enqueued=True):
        enqueued_status = TaskStatus.ENQUEUED.value
        if amr_id is not None:
            url = (f'{TESTBED_EMULATOR_APP_SERVER_IP}/amrmissions/?status={enqueued_status}&amr_id={amr_id.value}&ordering=enqueue_time')
        else:
            url = (f'{TESTBED_EMULATOR_APP_SERVER_IP}/amrmissions/?status={enqueued_status}&ordering=enqueue_time')
        response = requests.get(url)
        # self.get_logger().info(f'response.json(): {response.json()}')
        if response.status_code != 200:
            raise Exception(f'Error occurred while fetching AMRMissions: {response.text}')
        # If no enqueued tasks
        if len(response.json()) == 0:
            return None
        if only_fetch_first_enqueued:
            return response.json()[0]
        return response.json()
    
    # Function to update the status of an AMR mission
    def update_amr_mission_status(self, mission_url, new_status):
        # Prepare the data to be patched
        patch_data = {'status': new_status.value}
        if new_status == TaskStatus.RUNNING:
            patch_data['start_time'] = datetime.now().isoformat()
        if new_status == TaskStatus.COMPLETED:
            patch_data['end_time'] = datetime.now().isoformat()

        self.get_logger().info(f'mission_url: {mission_url}, patch_data: {patch_data}')


        # Send the PATCH request
        response = requests.patch(mission_url, json=patch_data)

        # Check if the request was successful
        if response.status_code != 200:
            raise Exception(f'Error occurred while updating AMRMission: {response.text}')

        # Return the response JSON if needed
        return response.json()

    def send_mission_control_action(self, action_goal_params, undock_flags):
        self.get_logger().info(f"action_goal_params: {action_goal_params}")
        self.get_logger().info(f"undock_flags: {undock_flags}")
        goal_msg = MissionControl.Goal()
        goal_msg.robot_specific_dock_ids = action_goal_params
        goal_msg.robot_specific_undock_flags = undock_flags
        self.action_client.wait_for_server()
        print(f"goal_msg: {goal_msg}")
        print(f"goal_msg.robot_specific_dock_ids: {goal_msg.robot_specific_dock_ids}")
        print(f"goal_msg.robot_specific_undock_flags: {goal_msg.robot_specific_undock_flags}")
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_ongoing_amr_action_servers(self):
        # This is the sentinel action message for cancelling
        self.send_mission_control_action([CANCEL_SENTINEL_VALUE]*4, [CANCEL_SENTINEL_VALUE]*2)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def mark_missions_as_completed(self):
        for mission_url in self.amr_mission_urls.values():
            if mission_url is not None:
                self.get_logger().info(f'Marking mission_url: {mission_url} as COMPLETED')
                self.update_amr_mission_status(mission_url, TaskStatus.COMPLETED)
                # TODO[for demo]: Send POST request to Executor to signal task completion
        for amr_id in self.amr_mission_urls:
            self.amr_mission_urls[amr_id] = None
            self.amr_mission_goals[amr_id] = None
        

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Action completed with status code: {result.status_code}') 
        # Updates AMR
        if result.status_code == 200:
            self.mark_missions_as_completed()
        else:
            self.get_logger().error(f'Error message: {result.text}')
            # TODO(sid): Mark as failure in db
            pass

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
