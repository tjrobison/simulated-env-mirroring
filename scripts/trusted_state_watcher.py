#!/usr/bin/env python

import rospy

TRASH_LOC = {   'px':-6.31239, 'py':-0.025627, 'pz':0.235926,
                'ox':0.000878, 'oy':0, 'oz':0.026814, 'ow': 0.3
            }

LINEAR_TOLERANCE = 0.5
ANGULAR_TOLERANCE = 0.3

class TrustedStateWatcher:

    """
    Init the TrustedStateWatcher node with relevant subs/pubs
    """
    def __init__(self):
        self.overlay_flag = 1

        self.bot_pose_sub = rospy.Subscriber("/odom", Odometry, self.bot_pose_sub_callback)
        self.enable_overlay_pub = rospy.Publisher("/enable_overlay", Int32, queue_size=3)

    """
    Check the provided values of the current position of the robot against the position
    of the specified object, and whether they are within the tolerance values specified
    """
    def check_tolerance(self, x, y, w, cur_x, cur_y, cur_w):
        trusted_loc = False
        x_loc = cur_x < x + LINEAR_TOLERANCE or cur_x > x - LINEAR_TOLERANCE
        y_loc = cur_y < y + LINEAR_TOLERANCE or cur_y > y - LINEAR_TOLERANCE
        w_loc = cur_w < w + ANGULAR_TOLERANCE or cur_w > w - ANGULAR_TOLERANCE 

        if x_loc or y_loc or w_loc:
            trusted_loc = True

        return trusted_loc

    """
    Pull the relevant positional parameters from the robot's odom topic and compare them
    to an object's position. If the position is considered to be trusted, it publishes overlay_flag
    to indicate to the camera_view node to swap camera views
    """
    def bot_pose_sub_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ow = msg.pose.pose.orientation.w

        trusted_loc = check_tolerance(TRASH_LOC['px'], TRASH_LOC['px'], TRASH_LOC['px'], x, y, w)
        if trusted_loc:
            self.overlay_flag = 0
        else
            self.overlay_flag = 1

        self.enable_overlay_pub.publish(self.overlay_flag)


if __name__ == '__main__':
    rospy.init_node("trusted_state_watcher")
    sim_pose_sync = TrustedStateWatcher()
    rospy.spin()
