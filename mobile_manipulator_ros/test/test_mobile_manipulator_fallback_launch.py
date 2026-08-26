import unittest

from geometry_msgs.msg import Pose
from grasping_msgs.action import MoveToPose
from grasping_msgs.srv import CheckCartesianPoseFeasibility
import launch
import launch_ros.actions
import launch_testing.actions
from mobile_manipulator_msgs.action import ExecuteTaskPose
from mobile_manipulator_msgs.srv import CheckTaskFeasibility, FindFeasiblePose
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer


@pytest.mark.launch_test
def generate_test_description():
    server = launch_ros.actions.Node(
        package='mobile_manipulator_ros',
        executable='mobile_manipulator_server',
        name='mobile_manipulator_server',
        output='screen',
    )
    return launch.LaunchDescription([server, launch_testing.actions.ReadyToTest()]), {}


class TestMobileManipulatorFallbackLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('mobile_manipulator_fallback_test')
        self.node.create_service(
            CheckCartesianPoseFeasibility,
            'check_cartesian_pose_feasibility',
            self._handle_grasping_feasibility,
        )
        self.navigation_action = ActionServer(
            self.node,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self._execute_navigation,
        )
        self.move_arm_action = ActionServer(
            self.node,
            MoveToPose,
            'move_arm_to_pose',
            execute_callback=self._execute_arm,
        )

    def tearDown(self):
        self.navigation_action.destroy()
        self.move_arm_action.destroy()
        self.node.destroy_node()

    def _handle_grasping_feasibility(self, request, response):
        response.mode_used = request.mode
        response.planning_frame = request.frame_id
        response.planning_pose = request.pose
        response.planning_pose_valid = True
        if request.frame_id == 'base_link':
            response.feasible = True
            response.failure_reason = ''
            response.suggested_fallback = ''
            response.message = 'Projected target is feasible from the supplied base pose.'
            return response

        response.feasible = False
        response.failure_reason = 'workspace_area_violation'
        response.suggested_fallback = 'move_base_then_arm'
        response.message = 'Direct arm-only reach is not feasible from the current base pose.'
        return response

    def _execute_navigation(self, goal_handle):
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _execute_arm(self, goal_handle):
        result = MoveToPose.Result()
        result.success = True
        result.message = 'Arm motion completed successfully.'
        goal_handle.succeed()
        return result

    def test_fallback_loop_finds_candidate_and_executes(self):
        check_client = self.node.create_client(CheckTaskFeasibility, 'check_task_feasibility')
        find_client = self.node.create_client(FindFeasiblePose, 'find_feasible_pose')
        execute_client = ActionClient(self.node, ExecuteTaskPose, 'execute_task_pose')

        assert check_client.wait_for_service(timeout_sec=10.0)
        assert find_client.wait_for_service(timeout_sec=10.0)
        assert execute_client.wait_for_server(timeout_sec=10.0)

        request = CheckTaskFeasibility.Request()
        request.mode = 'mobile_manipulator'
        request.frame_id = 'map'
        request.pose = Pose()
        request.pose.position.x = 0.8
        request.pose.position.y = 0.0
        request.pose.position.z = 0.25
        request.pose.orientation.w = 1.0
        request.base_pose_valid = False
        request.base_frame_id = 'map'

        check_future = check_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, check_future, timeout_sec=10.0)
        check_response = check_future.result()
        assert check_response is not None
        assert check_response.feasible is False
        assert check_response.suggested_fallback == 'move_base_then_arm'

        find_request = FindFeasiblePose.Request()
        find_request.pose = request.pose
        find_request.frame_id = request.frame_id
        find_request.mode = request.mode
        find_request.base_pose_valid = request.base_pose_valid
        find_request.base_pose = request.base_pose
        find_request.base_frame_id = request.base_frame_id

        find_future = find_client.call_async(find_request)
        rclpy.spin_until_future_complete(self.node, find_future, timeout_sec=10.0)
        find_response = find_future.result()
        assert find_response is not None
        assert find_response.feasible is True
        assert find_response.base_target_pose_valid is True
        assert find_response.base_target_frame == 'map'

        execute_goal = ExecuteTaskPose.Goal()
        execute_goal.pose = request.pose
        execute_goal.frame_id = request.frame_id
        execute_goal.mode = request.mode
        execute_goal.validated = True
        execute_goal.approved_base_pose_valid = find_response.base_target_pose_valid
        execute_goal.approved_base_pose = find_response.base_target_pose
        execute_goal.approved_base_frame = find_response.base_target_frame

        send_future = execute_client.send_goal_async(execute_goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        assert goal_handle is not None
        assert goal_handle.accepted

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=20.0)
        wrapped_result = result_future.result()
        assert wrapped_result is not None
        assert wrapped_result.result.success is True
        assert wrapped_result.result.stage == 'completed'
        assert wrapped_result.result.base_target_pose_valid is True