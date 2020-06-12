#!/usr/bin/env python

import rospy

# Mongodb
from mongodb_store.message_store import MessageStoreProxy

# TF
import tf2_ros

# ROS message
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse


class TfDatabasePublisher(object):

    rate = 125
    transforms = None
    msg_store = None
    world = None

    def __init__(self, world_frame='world'):
        self.msg_store = MessageStoreProxy()
        service = rospy.Service('apps/database/reload', Empty, self._cb_db_reload)

        # Define the 'world' frame
        self.world = TransformStamped()
        self.world.header.frame_id = 'world'
        self.world.child_frame_id = world_frame
        self.world.transform.rotation.w = 1.0

        # Publish the database at start
        self.reload_db()

        # Make the program stay awake
        rospy.spin()

    def _cb_db_reload(self, req):
        self.reload_db()
        return EmptyResponse()

    def reload_db(self):
        try:
            self.read_database()
            self.broadcast_transforms()
            return True
        except Exception:
            return False

    def broadcast_transforms(self):
        # Acquire the current time for the world frame
        self.world.header.stamp = rospy.Time.now()
        self.transforms.append(self.world)

        # Publish all frames
        static_transform_broadcaster = tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster()
        static_transform_broadcaster.sendTransform(self.transforms)

    def read_database(self):
        self.transforms = []
        transforms_db = self.msg_store.query(TransformStamped._type)
        for transform in transforms_db:
            self.transforms.append(transform[0])
        rospy.loginfo('Got {0} transformations from database!'.format(len(self.transforms)))