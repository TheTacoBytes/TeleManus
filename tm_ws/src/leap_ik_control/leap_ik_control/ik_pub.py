#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import numpy as np
from numpy import pi
from leap import datatypes as ldt
from leap import Connection, Listener
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import time
import matplotlib.pyplot

# Constants
deg2rad = np.pi / 180
rad2deg = 180 / np.pi


def location_end_of_finger(hand: ldt.Hand, digit_idx: int) -> ldt.Vector:
    digit = hand.digits[digit_idx]
    return digit.distal.next_joint


def sub_vectors(v1: ldt.Vector, v2: ldt.Vector) -> list:
    return map(float.__sub__, v1, v2)


def fingers_pinching(thumb: ldt.Vector, index: ldt.Vector):
    diff = list(map(abs, sub_vectors(thumb, index)))
    return diff[0] < 40 and diff[1] < 40 and diff[2] < 40


class TrackingListener(Listener):
    def __init__(self):
        super().__init__()
        self.latest_hand_centroid = None
        self.hand_closed = False

    def on_tracking_event(self, event):
        self.hand_closed = False
        if len(event.hands) > 0:
            hand = event.hands[0]

            # Check for closed hand
            self.hand_closed = hand.grab_strength > 0.8
            self.latest_hand_centroid = self.convert_to_grid_coordinates(hand.palm.position)
        else:
            self.latest_hand_centroid = None

    def convert_to_grid_coordinates(self, vector):
        y = np.interp(vector.x, [-2*200, 2*200], [50, -50])
        x = np.interp(vector.z, [-2*300, 2*100], [50, 12])
        z = np.interp(vector.y, [300, 500], [5, 50])
        return (x, y, z)

    def calculate_centroid(self, points):
        if points:
            x_avg = np.mean([p[0] for p in points])
            y_avg = np.mean([p[1] for p in points])
            z_avg = np.mean([p[2] for p in points])
            return (x_avg, y_avg, z_avg)
        return None

    def get_latest_hand_centroid(self):
        return self.latest_hand_centroid


class ArmControlPublisher(Node):
    def __init__(self):
        super().__init__('arm_control_publisher')
        self.publisher = self.create_publisher(Int32MultiArray, 'arm_control_topic', 10)
        self.timer = self.create_timer(0.1, self.publish_angles)
        self.current_angles = [0, 0, 0, 0, 0]

    def publish_angles(self, a1=0, a2=0, a3=0, a4=0, a5=0):
        message = Int32MultiArray()
        message.data = [a1, a2, a3, a4, a5]
        self.publisher.publish(message)
        self.get_logger().info(f'Publishing: {message.data}')


def main(args=None):
    rclpy.init(args=args)
    arm_control_publisher = ArmControlPublisher()

    listener = TrackingListener()
    connection = Connection()
    connection.add_listener(listener)

    with connection.open():
        try:
            while rclpy.ok():
                position = listener.get_latest_hand_centroid()
                s6 = 150 if listener.hand_closed else 55

                if position is not None:
                    x, y, z = map(round, position, [2, 2, 2])
                    target_position = x, y, z

                    arm_chain = Chain(name='dofbot', links=[
                        OriginLink(),
                        URDFLink(name="s1", origin_translation=[0, 0, 15.24],
                                 origin_orientation=[0, 0, 0], rotation=[0, 0, 1], bounds=(-pi/2, pi/2)),
                        URDFLink(name="s2", origin_translation=[0, 0, 2.85],
                                 origin_orientation=[0, 0, 0], rotation=[0, 1, 0], bounds=(-0.01, pi/2)),
                        URDFLink(name="s3", origin_translation=[0, 0, 8.255],
                                 origin_orientation=[0, 0, 0], rotation=[0, 1, 0], bounds=(-0.01, pi/2)),
                        URDFLink(name="s4", origin_translation=[0, 0, 8.255],
                                 origin_orientation=[0, 0, 0], rotation=[0, 1, 0], bounds=(-0.01, pi/2)),
                        URDFLink(name="s5", origin_translation=[0, 0, 10],
                                 origin_orientation=[0, 0, 0], rotation=[0, 0, 0]),
                        URDFLink(name="gripperEnd", origin_translation=[0, 0, 3.81],
                                 origin_orientation=[0, -1.57, 0], rotation=[0, 0, 1]),
                    ])

                    if target_position[1] == 0:
                        target_position[1] = 0.1
                    if target_position[0] == 0:
                        target_position[0] = 0.1

                    ik_angles = arm_chain.inverse_kinematics(target_position)
                    correction = np.array([0, -pi/2, -pi/2, -pi/2, -pi/2, 0, 0]) if target_position[1] else np.zeros(7)
                    arm_angles = np.round(rad2deg * (ik_angles + correction))[1:5]

                    s1, s2, s3, s4 = map(int, abs(arm_angles))
                    arm_control_publisher.publish_angles(s1, s2, s3, s4, s6)

        except KeyboardInterrupt:
            pass
        finally:
            connection.remove_listener(listener)
            rclpy.shutdown()


if __name__ == '__main__':
    main()
