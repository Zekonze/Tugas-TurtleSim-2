#!/usr/bin/env python3
"""Virtual sensor that publishes distance to the four arena walls."""

import rospy
from turtlesim.msg import Pose

from turtle_mission.msg import WallSensor


class WallSensorNode:
    def __init__(self):
        self.arena_max = rospy.get_param("/mission/arena_max")
        self.caution = rospy.get_param("/mission/wall_caution")
        self.publisher = rospy.Publisher("/mission/wall_sensor", WallSensor, queue_size=1)
        self.subscriber = rospy.Subscriber("/turtle1/pose", Pose, self.on_pose, queue_size=1)

    def on_pose(self, pose):
        left = max(0.0, pose.x)
        right = max(0.0, self.arena_max - pose.x)
        bottom = max(0.0, pose.y)
        top = max(0.0, self.arena_max - pose.y)
        nearest = min(left, right, bottom, top)
        self.publisher.publish(
            WallSensor(
                stamp=rospy.Time.now(),
                left=left,
                right=right,
                bottom=bottom,
                top=top,
                nearest=nearest,
                near_wall=nearest < self.caution,
            )
        )


if __name__ == "__main__":
    rospy.init_node("wall_sensor")
    sensor = WallSensorNode()
    rospy.spin()
