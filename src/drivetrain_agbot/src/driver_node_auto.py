#!/usr/bin/env python

import rospy
import time

from std_msgs.msg import Int32, Int64, Float32MultiArray

_FREQUENCY = 20


def _clip(value, minimum, maximum):
    """Ensure value is between minimum and maximum."""

    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value


class Motor:
    def __init__(self, forward_pin, backward_pin):
        self._forward_pin = forward_pin
        self._backward_pin = backward_pin

    def move(self, speed_percent):
        speed = _clip(abs(speed_percent), 0, 100)

class Driver:
    def __init__(self):
        rospy.init_node('driver_auto')

        self._last_received = rospy.get_time()
        self._timeout = rospy.get_param('~timeout', 2)
        self._rate = rospy.get_param('~rate', 10)
        self._max_speed = rospy.get_param('~max_speed', 0.5)
        self._wheel_base = rospy.get_param('~wheel_base', 0.091)

        # Assign pins to motors. These may be distributed
        # differently depending on how you've built your robot
        self._left_motor = Motor(10, 9)
        self._right_motor = Motor(8, 7)
        self._left_speed_percent = 0
        self._right_speed_percent = 0

        # Setup subscriber for id, pos, and state msg
        rospy.Subscriber('id', Int32, self.id_received_callback)
        rospy.Subscriber('pos', Float32MultiArray, self.floatarr_received_callback)
        rospy.Subscriber('state', Int32, self.state_received_callback)

    def id_received_callback(self, message):
        """Handle new int command message."""

        self._last_received = rospy.get_time()
        print("ID: ")
        print(message.data)

    def state_received_callback(self, message):
        """Handle new int command message."""

        self._last_received = rospy.get_time()
        print("State: ")
        print(message.data)

    def floatarr_received_callback(self, message):
        """Handle new float arr command message."""

        self._last_received = rospy.get_time()
        print("Position: ")
        print(message.data)

        #TODO: calculate linear and angular values from pos array
        linear = 0 #temp
        angular = 0 #temp

#        print(linear)
#        print(angular)
    # This gets executed at class defintion time
	pub_du1 = rospy.Publisher('ch1', Int64, queue_size=10)
	pub_du2 = rospy.Publisher('ch2', Int64, queue_size=10)
	
	send_du_1 = 50*linear - 40*angular
	send_du_2 = 50*linear + 40*angular

	pub_du1.publish(send_du_1)
	pub_du2.publish(send_du_2)
	
        # Calculate wheel speeds in m/s
        left_speed = linear - angular*self._wheel_base/2
        right_speed = linear + angular*self._wheel_base/2

        # Ideally we'd now use the desired wheel speeds along
        # with data from wheel speed sensors to come up with the
        # power we need to apply to the wheels, but we don't have
        # wheel speed sensors. Instead, we'll simply convert m/s
        # into percent of maximum wheel speed, which gives us a
        # duty cycle that we can apply to each motor.
        self._left_speed_percent = (
            100 * left_speed/self._max_speed)
        self._right_speed_percent = (
            100 * right_speed/self._max_speed)

    def run(self):
        """The control loop of the driver."""

	rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            #TODO: do something with id and state - only move if same id and state is tracking
            # If we haven't received new commands for a while, we
            # may have lost contact with the commander-- stop
            # moving
            delay = rospy.get_time() - self._last_received
            if delay < self._timeout:
                self._left_motor.move(self._left_speed_percent)
                self._right_motor.move(self._right_speed_percent)
            else:
                self._left_motor.move(0)
                self._right_motor.move(0)
            rate.sleep()


def main():
	driver = Driver()
	# Run driver. This will block
	driver.run()

if __name__ == '__main__':
    main()

