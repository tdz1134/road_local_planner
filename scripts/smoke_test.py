#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""冒烟测试：灌入直路边界 + 全空闲栅格 + 低定位质量，
期望输出 mode=FOLLOW、有效路径、非零推荐速度。"""
import math
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Point32, Twist, Vector3
from std_msgs.msg import String
from road_local_planner.msg import Boundary, LocalizationQuality

STATUS = {"last": None}
PLANS = []


def on_status(msg):
    STATUS["last"] = msg.data


def on_plan(msg):
    PLANS.append(msg)


def make_boundary(y):
    b = Boundary()
    b.header.frame_id = "base_link"
    b.points = [Point32(x=1.0 + 39.0 * i / 19.0, y=y, z=0.0) for i in range(20)]
    b.confidence = 1.0
    return b


def make_map():
    m = OccupancyGrid()
    m.header.frame_id = "base_link"
    m.info.resolution = 0.1
    m.info.width = 400
    m.info.height = 200
    m.info.origin.position.x = 0.0
    m.info.origin.position.y = -10.0
    m.data = [0] * (400 * 200)  # 全空闲
    return m


def main():
    rospy.init_node("smoke_test")
    rospy.Subscriber("/road_local_planner/status", String, on_status)
    rospy.Subscriber("/road_local_planner/plan", Path, on_plan)
    p_l = rospy.Publisher("/road_local_planner/boundaries/left", Boundary, queue_size=1)
    p_r = rospy.Publisher("/road_local_planner/boundaries/right", Boundary, queue_size=1)
    p_m = rospy.Publisher("/road_local_planner/local_map", OccupancyGrid, queue_size=1)
    p_q = rospy.Publisher("/road_local_planner/localization/quality", LocalizationQuality, queue_size=1)
    p_o = rospy.Publisher("/road_local_planner/odom", Odometry, queue_size=1)
    rospy.sleep(1.0)

    rate = rospy.Rate(10)
    t0 = rospy.Time.now()
    while rospy.Time.now() - t0 < rospy.Duration(4.0) and not rospy.is_shutdown():
        p_l.publish(make_boundary(2.5))
        p_r.publish(make_boundary(-2.5))
        p_m.publish(make_map())
        q = LocalizationQuality()
        q.quality = 0.3  # 定位差 → 期望 FOLLOW
        p_q.publish(q)
        o = Odometry()
        o.twist.twist = Twist(linear=Vector3(5.0, 0, 0))
        p_o.publish(o)
        rate.sleep()

    print("last status :", STATUS["last"])
    print("plan msgs   :", len(PLANS))
    if PLANS:
        print("last path pts:", len(PLANS[-1].poses),
              "end:", PLANS[-1].poses[-1].pose.position if PLANS[-1].poses else None)


if __name__ == "__main__":
    main()
