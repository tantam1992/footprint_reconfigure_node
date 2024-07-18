#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client
from std_msgs.msg import String
from geometry_msgs.msg import Pose

# Define the different footprints
small_footprint = [[0.25, -0.40], [0.40, -0.25], [0.40, 0.25], [0.25, 0.40], [-0.97, 0.40], [-1.10, 0.10], [-1.10, -0.10], [-0.97, -0.40]]
big_footprint   = [[0.31, -0.50], [0.50, -0.31], [0.50, 0.31], [0.31, 0.50], [-0.97, 0.50], [-1.27, 0.42], [-1.27, -0.42], [-0.97, -0.50]]
mini_footprint  = [[0.15, -0.40], [0.15, -0.31], [0.15, 0.31], [0.15, 0.40], [-0.97, 0.40], [-1.10, 0.10], [-1.10, -0.10], [-0.97, -0.40]]

# Define docking areas
LG_dock_area_polygon    = [(-2.3715, 47.51), (-1.5262, 47.488), (-1.47, 56.148), (-2.1851, 56.092)]
FiveF_dock_area_polygon = [(1.4971, -19.281), (2.189, -19.467), (2.0321, -14.916), (1.4992, -14.899)]

class FootprintReconfigureNode:
    def __init__(self):
        rospy.init_node('footprint_reconfigure_node')

        self.current_pose = None
        self.fold_state = "UNKNOWN"
        self.last_non_dock_state = "UNKNOWN"

        rospy.loginfo("Footprint reconfigure node started")

        # Subscribe to fold_state and robot's pose
        rospy.Subscriber('/fold_state', String, self.fold_state_callback)
        rospy.Subscriber('robot_pose', Pose, self.pose_callback)

        # Dynamic Reconfigure client
        self.global_reconfigure_client = dynamic_reconfigure.client.Client('/move_base/global_costmap')
        self.local_reconfigure_client = dynamic_reconfigure.client.Client('/move_base/local_costmap')

    def fold_state_callback(self, msg):
        self.fold_state = msg.data
        if not self.check_is_inside_dock_area():
            self.last_non_dock_state = self.fold_state
        self.update_footprint()

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg
        # self.update_footprint()

    def update_footprint(self):
        if self.current_pose is None:
            return

        if self.check_is_inside_dock_area():
            new_footprint = mini_footprint
        else:
            if self.last_non_dock_state == "OPERATIONAL/READY":
                new_footprint = small_footprint
            else:
                new_footprint = big_footprint

        rospy.loginfo("Reconfiguring footprint to: {}".format(new_footprint))
        params = {'footprint': new_footprint}
        self.global_reconfigure_client.update_configuration(params)
        self.local_reconfigure_client.update_configuration(params)

    def check_is_inside_dock_area(self):
        if self.current_pose is None:
            return False

        x, y = self.current_pose.position.x, self.current_pose.position.y
        return (self.point_inside_polygon(x, y, LG_dock_area_polygon) or
                self.point_inside_polygon(x, y, FiveF_dock_area_polygon))

    def point_inside_polygon(self, x, y, vertices):
        n = len(vertices)
        inside = False
        p1x, p1y = vertices[0]
        for i in range(n + 1):
            p2x, p2y = vertices[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

if __name__ == '__main__':
    try:
        node = FootprintReconfigureNode()
        rate = rospy.Rate(5)  # 5Hz
        while not rospy.is_shutdown():
            node.update_footprint()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
