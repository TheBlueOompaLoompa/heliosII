import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from helios_action_interfaces.action import SetChannelType, Throttle, ArmESC, ServoAngle
from enum import Enum
import time

from std_msgs.msg import String

import board
from adafruit_servokit import ServoKit

class ChannelType(Enum):
    SERVO = 0
    ESC = 1


class ESCState(Enum):
    STOPPED = 0
    START_CALIBRATION = 1
    CALIBRATING = 2
    ARMING = 3
    ARMED = 4

class ServoActionServer(Node):
    def __init__(self):
        super().__init__('servo_action_server')
        self.get_logger().info('Starting ServoActionServer...')

        i2c = board.I2C()

        self.kit = ServoKit(channels=16, i2c=i2c)
        self.kit.continuous_servo[self.channel].set_pulse_width_range(0, max_value)
        
        self.channel_type = {}
        self.channel_value = {}
        self.esc_state = {}

        self._set_channel_type_action_server = ActionServer(
            self,
            SetChannelType,
            'set_channel_type',
            self.execute_channel_type_callback)

        self._set_arm_esc_action_server = ActionServer(
            self,
            ArmESC,
            'arm_esc',
            self.execute_arm_esc_callback)

    def execute_channel_type_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Executing Channel Type goal...')
        goal_handle.succeed()

        request_type = goal_handle.request.type
        channel = goal_handle.request.channel

        if request_type == ChannelType.ESC.value and request_type != self.channel_type[channel]:
            self.kit.continuous_servo[channel].set_pulse_width_range(0, max_value)
            self.esc_state[channel] = ESCState.STOPPED

        self.channel_value[channel] = 0
        self.channel_type[channel] = request_type

        result = SetChannelType.Result()
        result.channel = goal_handle.request.channel
        return result

        #for i in range(1, goal_handle.request.order + 1):
        #    feedback_msg.feedback = f'Progress: {i}/{goal_handle.request.order}'
        #    goal_handle.publish_feedback(feedback_msg)
        #    self.get_logger().info(feedback_msg.feedback)

    def execute_arm_esc_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Executing Arm ESC goal...')

        channel = goal_handle.request.channel

        if self.channel_type.get(channel) != None and self.channel_type[channel] == ChannelType.ESC.value:
            self.kit.continuous_servo[channel].set_pulse_width_range(self.min_value, self.max_value)
            self.esc_state[channel] = State.ARMING

            feedback_msg = ArmESC.Feedback()
            updates = 10
            for i in range(updates):
                sleep(6/updates)
                feedback_msg.remaining = 6-(i/6)
                feedback_msg.progress = i/updates
            self.state = State.ARMED
            goal_handle.succeed()
            result = SetChannelType.Result()
            result.channel = goal_handle.request.channel
            return result
        else:
            goal_handle.canceled()

        #for i in range(1, goal_handle.request.order + 1):
        #    feedback_msg.feedback = f'Progress: {i}/{goal_handle.request.order}'
        #    goal_handle.publish_feedback(feedback_msg)
        #    self.get_logger().info(feedback_msg.feedback)


def main(args=None):
    rclpy.init(args=args)
    action_server = ServoActionServer()

    rclpy.spin(action_server)
    #action_server.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()
