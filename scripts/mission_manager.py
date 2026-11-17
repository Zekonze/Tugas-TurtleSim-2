#!/usr/bin/env python3
"""Sequence the missions and manage the Hunter target lifecycle."""

import random
from math import hypot
from threading import Lock

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import Kill, SetPen, Spawn

from turtle_mission.msg import HuntScore, MissionStatus, MotionGoal
from turtle_mission.srv import StartMission, StartMissionResponse


class MissionError(Exception):
    """The mission could not finish using the available feedback or services."""


class MissionManager:
    MODES = ("all", "patrol", "explore", "hunter")
    COLORS = {
        "PATROL": (255, 185, 75),
        "EXPLORE": (100, 246, 183),
        "HUNTER": (244, 136, 222),
    }

    def __init__(self):
        self.config = rospy.get_param("/mission")
        self.default_mode = rospy.get_param("~mode")
        self.validate_config()
        self.pending_mode = self.default_mode if rospy.get_param("~autostart") else None
        self.running = False
        self.lock = Lock()

        self.last_status = None
        self.goal_id = 0
        self.target_serial = 0
        self.target_name = ""
        self.target_pose = None
        seed = self.config["hunter_seed"]
        self.random = random.Random(None if seed == -1 else seed)
        self.score = 0
        self.last_capture = -1.0
        self.scored_run = False

        self.goals = rospy.Publisher("/mission/goal", MotionGoal, queue_size=1, latch=True)
        self.scores = rospy.Publisher("/mission/score", HuntScore, queue_size=1, latch=True)
        self.status_sub = rospy.Subscriber(
            "/mission/status", MissionStatus, self.on_status, queue_size=1
        )
        self.start_service = rospy.Service("/start_mission", StartMission, self.on_start)
        self.spawn = rospy.ServiceProxy("/spawn", Spawn)
        self.kill = rospy.ServiceProxy("/kill", Kill)
        self.set_pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)

    def validate_config(self):
        config = self.config
        if self.default_mode not in self.MODES:
            raise ValueError("mode harus all, patrol, explore, atau hunter")
        if not 0.0 < config["waypoint_tolerance"] < 0.1:
            raise ValueError("waypoint_tolerance harus di antara 0 dan 0.1")
        if not 0.0 < config["capture_distance"] < 0.5 or config["hunter_targets"] < 1:
            raise ValueError("Set minimal satu target dan capture_distance di antara 0 dan 0.5")
        margin = config["spawn_margin"]
        arena_max = config["arena_max"]
        if not 0.0 < config["wall_stop"] < config["wall_caution"] < arena_max / 2:
            raise ValueError("Periksa urutan wall_stop, wall_caution, dan arena_max")
        if not config["wall_stop"] < margin < arena_max / 2:
            raise ValueError("spawn_margin harus di dalam arena dan di atas wall_stop")
        if not 0.0 < config["min_spawn_distance"] < arena_max / 2 - margin:
            raise ValueError("min_spawn_distance terlalu besar untuk area spawn")
        for name, minimum in (("patrol_waypoints", 4), ("explore_waypoints", 1)):
            points = config[name]
            if len(points) < minimum:
                raise ValueError(f"{name} perlu minimal {minimum} titik")
            for x, y in points:
                if min(x, y, arena_max - x, arena_max - y) <= config["wall_stop"]:
                    raise ValueError(f"Waypoint ({x}, {y}) berada di zona henti dinding")

    def on_status(self, status):
        self.last_status = status

    def on_start(self, request):
        mode = request.mode.strip().lower() or self.default_mode
        if mode not in self.MODES:
            return StartMissionResponse(False, "Mode: all, patrol, explore, hunter")
        with self.lock:
            if self.running or self.pending_mode is not None:
                return StartMissionResponse(False, "Misi sedang berjalan atau mengantre")
            self.pending_mode = mode
        return StartMissionResponse(True, f"Misi {mode} dijadwalkan")

    def send_goal(self, phase, point=None, tolerance=0.0, index=0, total=0):
        self.goal_id += 1
        x, y = point if point is not None else (0.0, 0.0)
        self.goals.publish(
            MotionGoal(
                goal_id=self.goal_id,
                phase=phase,
                waypoint_index=index,
                waypoint_total=total,
                x=x,
                y=y,
                tolerance=tolerance,
                active=point is not None,
            )
        )

    def fresh_status(self):
        status = self.last_status
        if status is not None and status.pose_ready:
            age = (rospy.Time.now() - status.stamp).to_sec()
            if 0.0 <= age <= self.config["feedback_timeout"]:
                return status
        return None

    def wait_for(self, read_value, timeout, failure):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            value = read_value()
            if value is not None:
                return value
            rate.sleep()
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException()
        raise MissionError(failure)

    def wait_for_arrival(self):
        expected_goal = self.goal_id

        def arrived():
            status = self.fresh_status()
            if status is not None and status.goal_id == expected_goal and status.arrived:
                return status
            return None

        return self.wait_for(arrived, 45.0, f"Tujuan {expected_goal} tidak tercapai")

    def change_pen(self, phase):
        try:
            rospy.wait_for_service("/turtle1/set_pen", timeout=2.0)
            self.set_pen(*self.COLORS[phase], 3, 0)
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn("Warna pena tidak berubah: %s", error)

    def visit_waypoints(self, phase, points):
        self.change_pen(phase)
        tolerance = self.config["waypoint_tolerance"]
        for index, point in enumerate(points, 1):
            rospy.loginfo("%s | Menuju waypoint %d/%d (%.2f, %.2f)", phase, index, len(points), *point)
            self.send_goal(phase, point, tolerance, index, len(points))
            status = self.wait_for_arrival()
            rospy.loginfo(
                "%s | Waypoint %d/%d tercapai, error %.3f",
                phase, index, len(points), status.distance_to_goal,
            )
            rospy.sleep(self.config["waypoint_pause"])

    def publish_score(self, complete=False):
        self.scores.publish(
            HuntScore(
                stamp=rospy.Time.now(),
                captured=self.score,
                target_total=self.config["hunter_targets"] if self.scored_run else 0,
                active_target=self.target_name,
                last_capture_distance=self.last_capture,
                complete=complete,
            )
        )

    def choose_spawn(self, status):
        margin = self.config["spawn_margin"]
        upper = self.config["arena_max"] - margin
        minimum = self.config["min_spawn_distance"]
        for _ in range(100):
            x, y = self.random.uniform(margin, upper), self.random.uniform(margin, upper)
            if hypot(x - status.x, y - status.y) >= minimum:
                return x, y
        raise MissionError("Tidak menemukan posisi spawn yang cukup jauh dari turtle1")

    def capture_one(self, index, total):
        if not self.target_name:
            status = self.wait_for(self.fresh_status, 3.0, "Feedback posisi turtle1 belum siap")
            x, y = self.choose_spawn(status)
            self.target_serial += 1
            name = f"hunt_target_{self.target_serial:02d}"
            self.target_name = self.spawn(x, y, self.random.uniform(-3.14, 3.14), name).name
            rospy.loginfo("HUNTER | Spawn %s di (%.2f, %.2f)", self.target_name, x, y)
        self.publish_score()
        self.target_pose = None

        def on_target_pose(pose):
            self.target_pose = (pose, rospy.Time.now())

        def fresh_target():
            sample = self.target_pose
            if sample is not None and (rospy.Time.now() - sample[1]).to_sec() <= 0.5:
                return sample[0]
            return None

        target_sub = rospy.Subscriber(f"/{self.target_name}/pose", Pose, on_target_pose, queue_size=1)
        try:
            target = self.wait_for(fresh_target, 3.0, "Pose target spawn tidak diterima")
            self.send_goal(
                "HUNTER", (target.x, target.y), self.config["capture_distance"], index, total
            )
            arrival = self.wait_for_arrival()
            target = self.wait_for(fresh_target, 1.0, "Pose target kedaluwarsa")
            distance = hypot(target.x - arrival.x, target.y - arrival.y)
            if distance >= 0.5:
                raise MissionError(f"Jarak target {distance:.3f}; syarat tangkap < 0.5")
            captured_name = self.target_name
            self.kill(captured_name)
            self.target_name = ""
            self.score += 1
            self.last_capture = distance
            self.publish_score(complete=self.score == total)
            rospy.loginfo("HUNTER | Tangkap %s, jarak %.3f, skor %d/%d", captured_name, distance, self.score, total)
        finally:
            target_sub.unregister()

    def hunt(self):
        rospy.wait_for_service("/spawn", timeout=5.0)
        rospy.wait_for_service("/kill", timeout=5.0)
        self.change_pen("HUNTER")
        total = self.config["hunter_targets"]
        for index in range(1, total + 1):
            self.capture_one(index, total)
            rospy.sleep(self.config["waypoint_pause"])

    def run_mission(self, mode):
        self.score = 0
        self.last_capture = -1.0
        self.scored_run = mode in ("all", "hunter")
        self.publish_score()
        self.wait_for(self.fresh_status, 8.0, "Turtlesim atau controller belum siap")
        rospy.sleep(self.config["start_delay"])
        rospy.loginfo("Misi dimulai | mode=%s", mode)

        if mode in ("all", "patrol"):
            self.visit_waypoints("PATROL", self.config["patrol_waypoints"])
            self.send_goal("PATROL_COMPLETE")
            rospy.loginfo("LEVEL 1 SELESAI | Turtle berhenti setelah waypoint terakhir")
            if mode == "all":
                rospy.sleep(self.config["phase_pause"])

        if mode in ("all", "explore"):
            self.visit_waypoints("EXPLORE", self.config["explore_waypoints"])
            self.send_goal("EXPLORE_COMPLETE")
            rospy.loginfo("LEVEL 2 SELESAI | Rute eksplorasi selesai")
            if mode == "all":
                rospy.sleep(self.config["phase_pause"])

        if self.scored_run:
            self.hunt()
            rospy.loginfo("LEVEL 3 SELESAI | Skor %d/%d", self.score, self.config["hunter_targets"])

        self.send_goal("COMPLETE")
        rospy.loginfo("MISI SELESAI | Turtle berhenti | mode=%s", mode)

    def run(self):
        self.send_goal("IDLE")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.lock:
                mode = self.pending_mode
                self.pending_mode = None
                self.running = mode is not None
            if mode is not None:
                try:
                    self.run_mission(mode)
                except rospy.ROSInterruptException:
                    raise
                except (MissionError, rospy.ROSException, rospy.ServiceException) as error:
                    self.send_goal("ERROR")
                    rospy.logerr("Misi dihentikan: %s", error)
                finally:
                    with self.lock:
                        self.running = False
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mission_manager")
    try:
        MissionManager().run()
    except rospy.ROSInterruptException:
        pass
