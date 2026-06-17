"""Route-level line-follow, three pick points, color-memory sorting mission."""

import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger, TriggerResponse


def private_param(name, default):
    if rospy.has_param("~" + name):
        return rospy.get_param("~" + name)
    return rospy.get_param("~line_pick_sort_mission/" + name, default)


def clamp(value, low, high):
    return max(low, min(high, value))


class LinePickSortMission:
    def __init__(self):
        self.cmd_pub = rospy.Publisher(private_param("cmd_vel_topic", "/cmd_vel"), Twist, queue_size=10)
        rospy.Subscriber(private_param("line_visible_topic", "/vision/line/visible"), Bool, self.visible_cb, queue_size=10)
        rospy.Subscriber(private_param("line_error_topic", "/vision/line/error"), Float32, self.error_cb, queue_size=10)
        rospy.Subscriber(private_param("intersection_topic", "/vision/intersection/detected"), Bool, self.intersection_cb, queue_size=10)

        self.pick_service_name = private_param("pick_service", "/visual_pick_place/pick")
        self.place_service_name = private_param("place_service", "/visual_pick_place/place")
        self.rate_hz = float(private_param("rate_hz", 20.0))
        self.auto_start = bool(private_param("auto_start", False))
        self.debounce_sec = float(private_param("intersection_debounce_sec", 1.2))
        self.lost_timeout = float(private_param("lost_timeout", 0.35))
        self.line_cfg = private_param("line_follow", {})
        self.turn_cfg = private_param("turns", {})
        self.route = private_param("route", [])

        self.visible = False
        self.error = 0.0
        self.intersection = False
        self.last_visible = rospy.Time(0)
        self.last_intersection = False
        self.last_count_time = rospy.Time(0)
        self.route_index = 0
        self.running = self.auto_start
        self.done = False

        rospy.Service("~start", Trigger, self.handle_start)
        rospy.Service("~stop", Trigger, self.handle_stop)
        rospy.loginfo("line_pick_sort_mission ready auto_start=%s route_len=%d", self.auto_start, len(self.route))

    def visible_cb(self, msg):
        self.visible = bool(msg.data)
        if self.visible:
            self.last_visible = rospy.Time.now()

    def error_cb(self, msg):
        self.error = float(msg.data)

    def intersection_cb(self, msg):
        self.intersection = bool(msg.data)

    def handle_start(self, _req):
        self.running = True
        self.done = False
        return TriggerResponse(True, "mission started")

    def handle_stop(self, _req):
        self.running = False
        self.stop_base()
        return TriggerResponse(True, "mission stopped")

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if self.running and not self.done:
                self.update()
            else:
                self.stop_base()
            rate.sleep()

    def update(self):
        now = rospy.Time.now()
        rising = self.intersection and not self.last_intersection
        self.last_intersection = self.intersection
        if rising and (self.last_count_time == rospy.Time(0) or (now - self.last_count_time).to_sec() >= self.debounce_sec):
            self.last_count_time = now
            self.execute_next_route_step()
            return
        self.publish_line_follow_cmd(now)

    def execute_next_route_step(self):
        if self.route_index >= len(self.route):
            self.finish("route complete")
            return
        step = self.route[self.route_index]
        self.route_index += 1
        rospy.loginfo("route step %s action=%s after=%s", step.get("id", self.route_index), step.get("action"), step.get("after", ""))
        self.stop_base()
        rospy.sleep(float(self.turn_cfg.get("settle_sec", 0.2)))

        action = step.get("action", "straight")
        if action == "left":
            self.turn("left")
        elif action == "right":
            self.turn("right")
        elif action == "pick":
            self.call_trigger(self.pick_service_name, "pick")
            if self.done:
                return
        elif action == "straight":
            pass

        after = step.get("after", "")
        if after == "turn_back":
            self.turn("back")
        elif after == "place":
            self.call_trigger(self.place_service_name, "place")
        elif after == "place_turn_back":
            self.call_trigger(self.place_service_name, "place")
            if self.done:
                return
            self.turn("back")
        elif after == "done":
            self.finish("done at final intersection")

    def call_trigger(self, service_name, label):
        rospy.loginfo("waiting for %s service %s", label, service_name)
        try:
            rospy.wait_for_service(service_name, timeout=8.0)
            resp = rospy.ServiceProxy(service_name, Trigger)()
        except rospy.ROSException as exc:
            self.finish("%s service unavailable: %s" % (label, exc))
            return
        except rospy.ServiceException as exc:
            self.finish("%s service call failed: %s" % (label, exc))
            return
        if not resp.success:
            self.finish("%s failed: %s" % (label, resp.message))
        else:
            rospy.loginfo("%s success: %s", label, resp.message)

    def publish_line_follow_cmd(self, now):
        cmd = Twist()
        if not self.visible or self.last_visible == rospy.Time(0) or (now - self.last_visible).to_sec() > self.lost_timeout:
            self.cmd_pub.publish(cmd)
            return
        err = 0.0 if abs(self.error) < float(self.line_cfg.get("error_deadband", 0.03)) else self.error
        angular = float(self.line_cfg.get("angular_sign", -1.0)) * float(self.line_cfg.get("kp_angular", 0.45)) * err
        cmd.linear.x = float(self.line_cfg.get("intersection_forward_speed", 0.025) if self.intersection else self.line_cfg.get("forward_speed", 0.055))
        cmd.angular.z = clamp(angular, -abs(float(self.line_cfg.get("max_angular_z", 0.35))), abs(float(self.line_cfg.get("max_angular_z", 0.35))))
        if math.isfinite(cmd.angular.z):
            self.cmd_pub.publish(cmd)

    def turn(self, direction):
        angular = abs(float(self.turn_cfg.get("angular_z", 0.32)))
        duration = float(self.turn_cfg.get("turn_back_sec" if direction == "back" else direction + "_sec", 1.15))
        sign = 1.0 if direction == "left" else -1.0
        if direction == "back":
            sign = 1.0
        cmd = Twist()
        cmd.angular.z = sign * angular
        end = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            self.cmd_pub.publish(cmd)
            rate.sleep()
        self.stop_base()
        rospy.sleep(float(self.turn_cfg.get("settle_sec", 0.2)))

    def finish(self, reason):
        rospy.logwarn("mission finished/stopped: %s", reason)
        self.done = True
        self.running = False
        self.stop_base()

    def stop_base(self):
        self.cmd_pub.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("line_pick_sort_mission")
    node = LinePickSortMission()
    rospy.on_shutdown(node.stop_base)
    node.spin()