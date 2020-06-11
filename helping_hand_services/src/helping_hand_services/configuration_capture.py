# Regex for finding namespaces
import re

# Numpy
import numpy as np

# TF
import tf
import tf2_ros
import tf2_geometry_msgs

#ROS:
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float64, UInt8, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform, TransformStamped, Point, Quaternion, Pose, PoseStamped

# Service messages
from helping_hand_msgs.srv import CaptureTF, CaptureTFRequest, CaptureTFResponse
from helping_hand_msgs.srv import CaptureJoint, CaptureJointRequest, CaptureJointResponse
from helping_hand_msgs.srv import CaptureTopic, CaptureTopicRequest, CaptureTopicResponse
from helping_hand_msgs.srv import CapturePose, CapturePoseRequest, CapturePoseResponse


# Mongo DB
from mongodb_store.message_store import MessageStoreProxy


class ConfigurationCapture(object):
    
    # TF buffer
    tf2_buffer = None

    # Database class
    _msg_store = None


    def __init__(self):
        
        rospy.init_node('configuration_services')

        # TF
        self.tf2_buffer = tf2_ros.Buffer()
        self.tf2_listener = tf2_ros.TransformListener(self.tf2_buffer)

        # Create database proxy
        self._msg_store = MessageStoreProxy()

        # Create database reload service
        try:
            rospy.wait_for_service('/apps/database/reload', 0.1)
            self._database_service = rospy.ServiceProxy('/apps/database/reload', Empty)
        except Exception as e:
            rospy.logerr('Could not initialize {}'.format(rospy.get_name()))
            rospy.loginfo('Exceptions: {}'.format(e))

            return 0

        # Define the service proxies
        try:
            save_tf_srvs = rospy.Service('tf_capture', CaptureTF, self._handle_tf_save)
            save_joint_srvs = rospy.Service('joint_capture', CaptureJoint, self._handle_joint_save)
        except Exception as e:
            rospy.logerr('Could not initialize {}'.format(rospy.get_name()))
            rospy.loginfo('Exceptions: {}'.format(e))

            return 0
        
        # Notify the user the node is running
        rospy.loginfo("Hepling hand is running!")

        rospy.spin()

    def _handle_joint_save(self, req):
        try:
            # Display what you are doing
            rospy.loginfo("Storing the provided joint configuration as <{}> into the database ...".format(
                req.entry_name
            ))

            # Save to DB
            self._save_to_db(req.joints, req.entry_name)
            
            # Return the success
            return CaptureJointResponse(message='Frame saved', success=True)
        except Exception as e:
            # Handle exceptions
            rospy.logerr("Failed to handle request with exception:\n{}".format(e))
            return CaptureJointResponse(message='Failed with exception:\n{}'.format(e), success=False)

    def _handle_pose_save(self, req):
        # Work in progress, not implemented yet
        pass

    def _handle_topic_save(self, req):
        # Work in progress, not implemented yet
        pass

    def _handle_tf_save(self, req):
        try:
            # Retrieve the transformation from the TF buffer
            resulting_frame = self.tf2_buffer.lookup_transform(req.from_frame, req.to_frame, rospy.Time())

            # Set the tranformation's ID to the desired one
            resulting_frame.child_frame_id = req.new_frame_name

            # Display what you are doing
            rospy.loginfo("Storing transformation from <{0}> to <{1}> with the new name <{2}> into the database ...".format(
                req.from_frame, req.to_frame, req.new_frame_name
            ))

            # Save to DB
            self._save_to_db(resulting_frame, req.new_frame_name)

            # Return the success
            return CaptureTFResponse(message='Frame saved', success=True)
        except Exception as e:
            # Handle exceptions
            rospy.logerr("Failed to handle request with exception:\n{}".format(e))
            return CaptureTFResponse(message='Failed with exception:\n{}'.format(e), success=False)

    def _save_to_db(self, entry, name):
        # If you cannot update an existing entry, make a new one
        if not(self._msg_store.update_named(name, entry).success):
            self._msg_store.insert_named(name, entry)
            rospy.loginfo('Entry <{}> inserted !!'.format(name))
        else:
            rospy.loginfo('Entry <{}> updated !!'.format(name))


        # Tell the service to reload all the TFs
        self._database_service()


