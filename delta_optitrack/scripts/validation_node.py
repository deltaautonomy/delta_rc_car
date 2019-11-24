#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Heethesh Vhavle
Email   : heethesh@cmu.edu
Version : 1.0.0
Date    : Nov 24, 2019
'''

# Python 2/3 compatibility
from __future__ import print_function, absolute_import, division

# External modules
import numpy as np
import matplotlib.pyplot as plt

# ROS modules
import rospy
import message_filters

# ROS messages
from geometry_msgs.msg import PoseStamped
from delta_msgs.msg import Track, TrackArray

# Global objects
STOP_FLAG = False
tf_listener = None

# Frames
RADAR_FRAME = 'ti_mmwave'
EGO_VEHICLE_FRAME = 'rviz'


########################### Functions ###########################


def convert_tracks_to_numpy(msg):   
    tracks = [[track.x, track.y, track.vx, track.vy] for track in msg.tracks]
    return np.array(tracks)


def convert_pose_to_numpy(msg):
    pose = [msg.pose.point.x, msg.pose.point.y, msg.pose.point.z]
    return np.array(pose)


def find_nearest_pose(src, dst):
    dists = np.sum((src[:, :2] - dst[:2]) ** 2, axis=1)
    return src[np.argmin(dists)]


########################### Classes ###########################


class OptiTrackData:
    def __init__(self, frame):
        self.frame = frame
        self.pose = None
        self.velocity = None
        self.timestamp = None
        self.first_time = True

    @property
    def state(self):
        return np.r_[self.pose[:2], self.velocity[:2]]

    def update(self, pose_msg):
        pose = convert_pose_to_numpy(pose_msg)
        timestamp = pose_msg.header.stamp

        if not self.first_time:
            self.estimate_velocity(pose, timestamp)

        self.pose = pose
        self.timestamp = timestamp
        self.first_time = False

    def estimate_velocity(self, pose, timestamp):
        dx = pose - self.pose
        dt = timestamp - self.timestamp
        self.velocity = dx / dt.to_secs()


class OptiTrackValidation:
    def __init__(self):
        self.position_errors = []
        self.velocity_errors = []
        self.rc_car = OptiTrackData('rc_car')
        self.oncoming_car = OptiTrackData('oncoming_car')

    def tracks_callback(self, tracks_msg):
        # Node stop has been requested
        if STOP_FLAG: return

        # Associate nearest detections with the ground truth pose
        tracks = convert_tracks_to_numpy(tracks_msg)
        detected_car = find_nearest_pose(tracks, self.oncoming_car.state)

        # Store errors
        errors = (detected_car - self.oncoming_car.state) ** 2
        self.position_errors.append(np.sum(errors[:2]))
        self.velocity_errors.append(np.sum(errors[2:]))

    def shutdown_hook(self):
        global STOP_FLAG
        STOP_FLAG = True
        time.sleep(3)

        print('\n\033[95m' + '*' * 25 + ' Delta OptiTrack Validation Results ' + '*' * 25 + '\033[00m\n')
        print('Position RMSE: %.3f m', np.sqrt(np.mean(self.position_errors)))
        print('Velocity RMSE: %.3f m/s', np.sqrt(np.mean(self.velocity_errors)))


def run(**kwargs):
    # Start node
    rospy.init_node('optitrack_validation', anonymous=False)
    rospy.loginfo('Current PID: [%d]' % os.getpid())

    # Handle params and topics
    tracks = rospy.get_param('~tracks', '/delta/tracking_fusion/tracker/tracks')
    rc_car_pose = rospy.get_param('~rc_car_pose', '/vrpn_client_node/rc_car/pose ')
    oncoming_pose = rospy.get_param('~oncoming_pose', '/vrpn_client_node/oncoming_car/pose ')

    # Display params and topics
    rospy.loginfo('TrackArray topic: %s' % tracks)
    rospy.loginfo('RC Car Pose topic: %s' % rc_car_pose)
    rospy.loginfo('Oncoming Car Pose topic: %s' % oncoming_pose)

    # OptiTrack validator
    validator = OptiTrackValidation()

    # Subscribe to topics
    rospy.Subscriber(tracks, TrackArray, validator.tracks_callback)
    rospy.Subscriber(rc_car_pose, PoseStamped, validator.rc_car.update)
    rospy.Subscriber(oncoming_pose, PoseStamped, validator.oncoming_car.update)

    # Shutdown hook
    rospy.on_shutdown(validator.shutdown_hook)

    # Keep python from exiting until this node is stopped
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down')


if __name__ == '__main__':
    # Start OptiTrack validation node
    run()
