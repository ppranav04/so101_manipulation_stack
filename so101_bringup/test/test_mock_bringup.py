"""Launch test: SO-101 mock bringup starts its controllers and executes a trajectory."""

import time
import unittest

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


EXPECTED_CONTROLLERS = [
    'joint_state_broadcaster',
    'joint_trajectory_controller',
    'parallel_gripper_controller',
]
ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


@pytest.mark.launch_test
def generate_test_description():
    """Start the real bringup launch file on mock hardware, without RViz."""
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('so101_bringup'), 'launch', 'so101.launch.py'])),
        launch_arguments={'hw': 'mock', 'rviz': 'false'}.items(),
    )
    return LaunchDescription([
        bringup,
        launch_testing.actions.ReadyToTest(),
    ])


class TestMockBringup(unittest.TestCase):
    """Checks run while the bringup is alive."""

    @classmethod
    def setUpClass(cls):
        """Start rclpy once for the whole file."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Stop rclpy once at the end."""
        rclpy.shutdown()

    def setUp(self):
        """Give each test a fresh node."""
        self.node = rclpy.create_node('test_mock_bringup')

    def tearDown(self):
        """Destroy the per-test node."""
        self.node.destroy_node()

    def _wait_for_active_controllers(self, timeout_sec=30.0):
        """Poll list_controllers until all expected controllers are active or time runs out."""
        client = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(
            client.wait_for_service(timeout_sec=timeout_sec),
            'controller_manager/list_controllers service never appeared')

        states = {}
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            if future.result() is not None:
                states = {c.name: c.state for c in future.result().controller}
                if all(states.get(name) == 'active' for name in EXPECTED_CONTROLLERS):
                    return states
            time.sleep(0.5)
        return states

    def test_controllers_active(self):
        """All three controllers reach the active state."""
        states = self._wait_for_active_controllers()
        for name in EXPECTED_CONTROLLERS:
            self.assertEqual(
                states.get(name), 'active',
                f'{name} not active; controller states were: {states}')

    def test_trajectory_goal_succeeds(self):
        """A single-point arm trajectory is accepted and finishes with SUCCEEDED."""
        self._wait_for_active_controllers()

        client = ActionClient(
            self.node, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        self.assertTrue(
            client.wait_for_server(timeout_sec=10.0),
            'follow_joint_trajectory action server never appeared')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = [0.3, -0.3, 0.3, 0.2, 0.0]
        point.time_from_start = Duration(sec=2)
        goal.trajectory.points = [point]

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        self.assertIsNotNone(goal_handle, 'no response to goal request')
        self.assertTrue(goal_handle.accepted, 'goal was rejected')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=10.0)
        response = result_future.result()
        self.assertIsNotNone(response, 'no result within 10 s')
        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(
            response.result.error_code, FollowJointTrajectory.Result.SUCCESSFUL,
            f'error: {response.result.error_string}')

        client.destroy()
