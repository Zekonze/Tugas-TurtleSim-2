#!/usr/bin/env python3
"""Closed-loop proportional controller for turtle1."""

from math import atan2, cos, hypot, sin

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from turtle_mission.msg import MissionStatus, MotionGoal, WallSensor


class MotionController:
    def __init__(self):
        config = rospy.get_param("/mission")
        self.k_linear = config["k_linear"]
        self.k_angular = config["k_angular"]
        self.max_linear = config["max_linear"]
        self.max_angular = config["max_angular"]
        self.heading_gate = config["heading_gate"]
        self.wall_stop = config["wall_stop"]
        self.wall_caution = config["wall_caution"]
        self.feedback_timeout = rospy.Duration(config["feedback_timeout"])
        self.control_hz = config["control_hz"]

        self.goal = MotionGoal(phase="IDLE")
        self.pose_sample = None
        self.wall = None
        self.velocity = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
        self.status = rospy.Publisher("/mission/status", MissionStatus, queue_size=1)
        self.pose_sub = rospy.Subscriber("/turtle1/pose", Pose, self.on_pose, queue_size=1)
        self.goal_sub = rospy.Subscriber("/mission/goal", MotionGoal, self.on_goal, queue_size=1)
        self.wall_sub = rospy.Subscriber(
            "/mission/wall_sensor", WallSensor, self.on_wall, queue_size=1
        )
        rospy.on_shutdown(self.stop)

    def on_pose(self, pose):
        self.pose_sample = (pose, rospy.Time.now())

    def on_goal(self, goal):
        self.goal = goal

    def on_wall(self, wall):
        self.wall = wall

    def stop(self):
        self.velocity.publish(Twist())

    def fresh(self, now, stamp):
        return rospy.Duration(0) <= now - stamp <= self.feedback_timeout

    def safe_linear_speed(self, speed, heading, wall):
        if not wall.near_wall:
            return speed

        # Limit the speed component toward each wall independently.
        for direction, lower, upper in (
            (cos(heading), wall.left, wall.right),
            (sin(heading), wall.bottom, wall.top),
        ):
            component = abs(direction)
            if component < 1e-6:
                continue
            distance = upper if direction > 0.0 else lower
            if distance < self.wall_caution:
                factor = max(0.0, distance - self.wall_stop)
                factor /= self.wall_caution - self.wall_stop
                speed = min(speed, self.max_linear * factor / component)
        return speed

    def step(self):
        now = rospy.Time.now()
        goal = self.goal
        pose_sample = self.pose_sample
        wall = self.wall
        command = Twist()
        status = MissionStatus(
            stamp=now,
            goal_id=goal.goal_id,
            phase=goal.phase,
            waypoint_index=goal.waypoint_index,
            waypoint_total=goal.waypoint_total,
            active=goal.active,
            distance_to_goal=-1.0,
        )

        if pose_sample is not None and self.fresh(now, pose_sample[1]):
            pose = pose_sample[0]
            status.pose_ready = True
            status.x, status.y = pose.x, pose.y
            if goal.active:
                dx, dy = goal.x - pose.x, goal.y - pose.y
                distance = hypot(dx, dy)
                status.distance_to_goal = distance
                if distance < goal.tolerance:
                    # Wait for pose feedback to confirm that the zero command took effect.
                    status.arrived = abs(pose.linear_velocity) < 0.01 and abs(
                        pose.angular_velocity
                    ) < 0.01
                elif wall is not None and self.fresh(now, wall.stamp):
                    heading_error = atan2(dy, dx) - pose.theta
                    heading_error = atan2(sin(heading_error), cos(heading_error))
                    angular = self.k_angular * heading_error
                    command.angular.z = max(-self.max_angular, min(self.max_angular, angular))
                    if abs(heading_error) < self.heading_gate:
                        linear = min(self.max_linear, self.k_linear * distance)
                        command.linear.x = self.safe_linear_speed(
                            linear * cos(heading_error), pose.theta, wall
                        )

        self.velocity.publish(command)
        self.status.publish(status)

    def run(self):
        rate = rospy.Rate(self.control_hz)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("motion_controller")
    try:
        MotionController().run()
    except rospy.ROSInterruptException:
        pass
