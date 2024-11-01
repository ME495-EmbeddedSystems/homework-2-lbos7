import rclpy
from rclpy.node import Node
import unittest
import pytest
from launch import LaunchDescription
from launch_testing.actions import ReadyToTest
from launch_testing_ros.wait_for_topics import WaitForTopics
from geometry_msgs.msg import Twist
# Mark the launch description generation as a rostest
# Also, it's called generate_test_description() for a test
# But it still returns a LaunchDescription
@pytest.mark.rostest
def generate_test_description():

    turtle_robot_action = Node(package='turtle_brick',
                               executable='turtle_robot')

    return (
        LaunchDescription([
            turtle_robot_action,
            ReadyToTest()
            ]),
            {'turtle_robot': turtle_robot_action}
            )

# The above returns the launch description. Now it's time for the test
# The goal is essentially to create a node that can then be used in all tests to
# call services and subscribe/publish messages
# unlike a regular node, it is often useful to not subclass node but rather
# just create it. It is also useful (and necessary) to spin_once() as needed
class TestFrequency(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Runs one time, when the testcase is loaded"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """ Runs one time, when testcase is unloaded"""
        rclpy.shutdown()

    def setUp(self):
        """Runs before every test"""
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        """Runs after every test"""
        self.node.destroy_node()

    def test_frequency(self, launch_service, node, proc_output):
        """In UnitTest, any function starting with "test" is run as a test

          Args:
              launch_service - information about the launch
              myaction - this was passed in when we created the description
              proc_output - this object streams the output (if any) from the running process
        """
        wait_for_topics = WaitForTopics([('turtle1/cmd_vel', Twist)], timeout=5.0)
        assert wait_for_topics.wait()
        self.freqs = []
        self.timestamps = []
        while len(self.timestamps) < 101:
            rclpy.spin_once(node)
            time = self.node.get_clock().now().to_msg()
            if len(self.timestamps) > 0:
                self.freqs.append(1/(time - self.timestamps[-1]))
            self.timestamps.append(time)

        self.assertEqual(sum(self.freqs)/len(self.freqs) < 105)
        self.assertEqual(sum(self.freqs)/len(self.freqs) > 95)