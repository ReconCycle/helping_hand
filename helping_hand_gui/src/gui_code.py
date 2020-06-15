# A DMP recording GUI
#
# Author: Timotej Gaspar
# E-mail: timotej.gaspar@ijs.si
#

import os
import sys

from functools import partial

# Qt stuff
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QRegExp
# This won't work in Kinetic:
# from python_qt_binding.QtGui import QWidget
# Kinetic uses Qt 5:
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QHeaderView
from python_qt_binding.QtWidgets import QTextEdit
from python_qt_binding.QtWidgets import QAbstractItemView
from python_qt_binding.QtWidgets import QListWidget, QListWidgetItem
from python_qt_binding.QtWidgets import QLineEdit
from python_qt_binding.QtWidgets import QCheckBox
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QComboBox
from python_qt_binding.QtWidgets import QTableWidget, QTableWidgetItem
from python_qt_binding.QtCore import pyqtSignal, pyqtSlot
from python_qt_binding.QtCore import Qt

# Regex for namespace extraction
import re

# ROS stuff
import rospy
import rospkg
import rostopic
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from std_srvs.srv import Empty
from robot_module_msgs.msg import JointSpaceDMP
from robot_module_msgs.msg import CartesianSpaceDMP

# Database saving services
from helping_hand_msgs.srv import CaptureTF, CaptureTFRequest
from helping_hand_msgs.srv import CaptureJoint, CaptureJointRequest
from helping_hand_msgs.srv import CaptureDMP, CaptureDMPRequest


# DMP Service
from dmp_record_tool_msgs.srv import EncodeTrajectoryToDMP, EncodeTrajectoryToDMPRequest
import actionlib

# YAML
import yaml

# TF
import tf2_ros
import tf

import time
from datetime import datetime

# import pyaudio
# import numpy as np


# DMP Message definition
# import robot_module_msgs.msg
# Action messages
import dmp_record_tool_msgs.msg

# Mongo DB
from mongodb_store.message_store import MessageStoreProxy

# Button pins
BUTTON_1_PIN = 0
BUTTON_2_PIN = 1
BUTTON_A_PIN = 2
BUTTON_B_PIN = 3

# Gravity compensation
GRAVCOMP_ON = True
GRAVCOMP_OFF = False

# Desired message types
DESIRED_MSGS = ['geometry_msgs/PoseStamped', 'sensor_msgs/JointState']


def rising_edge(old_val, new_val):
    if old_val - new_val == -1:
        return True
    else:
        return False


def falling_edge(old_val, new_val):
    if old_val - new_val == 1:
        return True
    else:
        return False


def traj_duration_sec(traj):
    t_start = traj[0].header.stamp
    t_end = traj[-1].header.stamp

    return (t_end - t_start).to_sec()


# Helper class
class StatusReport(dict):
    def __init__(self, dic, gui):
        self.gui = gui
        super(StatusReport, self).__init__(dic)

    def __setitem__(self, item, value):
        super(StatusReport, self).__setitem__(item, value)
        self.gui._print_status()


class HelpingHandGUI(Plugin):

    # Various member variables
    # _topic_subscriber = None
    # _topic_good = False
    # _digitalinput_subscriber = None
    # _namespace = None
    _available_ns = []
    _conf_yaml = None
    _trigger_sources = []
    _data_buffer = {}
    _refresh_data_table_contents_sig = pyqtSignal()
    # _pattern = re.compile(r'\/[^/]+\/')
    # _digitalinput = None
    # _gravcomp_service = None
    # _dmp_action_server_client = None
    # _topic_name = None
    # _num_weights = 0
    # _recorded_dmp = None
    # _msg_store = None
    # _database_service = None
    # _dmp_save_name = None
    # _pyaudio = None
    # _stream = None
    # _beep = None
    # _joint_space = None
    # _cartesian_space = None
    # _available_frames = []

    # DMP trajectory buffer
    _joint_traj = []

    # DMP encoding service
    _dmp_ecoder = None

    # Logging
    _init_status_report = {
        'status': 'idle',
        'last_entry': 'None',
        'last_type': 'None',
    }

    # Logics
    _old_gravbutton = 0
    _old_rec_button = 0
    _recording = False
    _gravComp = False

    # Some signal
    _dmp_recording_started_signal = pyqtSignal()
    _dmp_recording_stopped_signal = pyqtSignal()

    # TF2
    _tf2_buffer = tf2_ros.Buffer()
    _tf2_listener = tf2_ros.TransformListener(_tf2_buffer)
    _tf_listener = tf.TransformListener()

    # Available topics
    _available_topics_array = []
    _available_topics_dict = dict()
    _old_index = 0

    def __init__(self, context):
        super(HelpingHandGUI, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('HelpingHandGUI')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        # args, unknowns = parser.parse_known_args(context.argv())
        # if not args.quiet:
        #     rospy.logdebug 'arguments: ', args
        #     rospy.logdebug 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('helping_hand_gui'), 'qt', 'helpinghand.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        # self._widget.setObjectName('DMPRecordToolGUIUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Populate configuration table's headers
        self._conf_table = self._widget.findChild(QTableWidget, 'configTable')
        self._conf_table.setHorizontalHeaderLabels((
            'Trigger Name',
            'Robot Namespace',
            'Trigger Topic',
            'Trigger Type',
            'Trigger Value',
            'Trigger Callback'))

        # Populate data table's headers
        self._data_table = self._widget.findChild(QTableWidget, 'data_table')
        self._data_table.setHorizontalHeaderLabels((
            'Timestamp',
            'Trigger Conf',
            'Trigger Reason',
            'Data Type'))

        header = self._data_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)

        # Hide the 5th column that contains data identifiers
        self._data_table.setColumnCount(self._data_table.columnCount()+1)
        self._data_table.setColumnHidden(self._data_table.columnCount()-1, True)

        # Connect the refresh signal with the update function
        self._refresh_data_table_contents_sig.connect(self._update_data_table)

        # Connect the item selected signal to the display details
        self._data_table.itemSelectionChanged.connect(self._display_item_details)

        # Parse config
        self._parse_yaml_config()

        # Show conf in table
        self._write_conf_to_table()

        # Display what we are listening to
        self._display_addresses()

        # Print initial status report
        self.status_report = StatusReport(self._init_status_report, self)
        self._print_status()

        # DMP Encoding service
        self._dmp_ecoder = rospy.ServiceProxy('/encode_traj_to_dmp', EncodeTrajectoryToDMP)

        # Save button widget
        save_button = self._widget.findChild(QPushButton, 'save_button')
        save_button.clicked.connect(partial(self._save_button_pressed, save_button))

        # Create database proxy
        self._msg_store = MessageStoreProxy()

        # Create the service proxies for into saving databse
        self._tf_capture_srv = rospy.ServiceProxy('tf_capture', CaptureTF)
        self._joint_capture_srv = rospy.ServiceProxy('joint_capture', CaptureJoint)
        self._dmp_capture_srv = rospy.ServiceProxy('dmp_capture', CaptureDMP)

        # Make sure the services are running !!
        try:
            self._tf_capture_srv.wait_for_service(0.5)
            self._joint_capture_srv.wait_for_service(0.5)
            self._dmp_capture_srv.wait_for_service(0.5)
        except Exception as e:
            rospy.logerr("Could not establish a connection to services. See exception:\n{}".format(e))
            exit()


    def _print_status(self):
        status_text_widget = self._widget.findChild(QTextEdit, 'statusWindow')

        status_text = \
            "Status: {0}\n"\
            "Last saved entry: {1}\n"\
            "Last entry type: {2}".format(
                self.status_report['status'],
                self.status_report['last_entry'],
                self.status_report['last_type']
            )

        status_text_widget.setText(status_text)

    # YAML config parser
    def _parse_yaml_config(self):
        conf_file = os.path.join(rospkg.RosPack().get_path('helping_hand_gui'), 'conf', 'triggers_conf.yml')
        self._conf_yaml = yaml.load(open(conf_file), Loader=yaml.FullLoader)

        tmp_ns = []
        for conf_key, conf_val in self._conf_yaml.items():
            tmp_ns.append(conf_val['robot_ns'])
            if conf_val['trig_type'] == 'rising_edge':
                conf_val['subscriber'] = rospy.Subscriber(conf_val['trig_topic'], Bool, callback=self._rising_edge_cb, callback_args=(conf_key))
            elif conf_val['trig_type'] == 'falling_edge':
                conf_val['subscriber'] = rospy.Subscriber(conf_val['trig_topic'], Bool, callback=self._falling_edge_cb, callback_args=(conf_key))
            elif conf_val['trig_type'] == 'hold':
                conf_val['subscriber'] = rospy.Subscriber(conf_val['trig_topic'], Bool, callback=self._hold_cb, callback_args=(conf_key))
            else:
                self.status_report['status'] = "Error parsing trigger types."

            if conf_val['trig_callback'] == 'joint_save':
                joint_topic_addr = conf_val['robot_ns'] + conf_val['joint_topic']
                conf_val['subscriber'] = rospy.Subscriber(joint_topic_addr, JointState, callback=self._joint_state_cb, callback_args=(conf_key))
            if conf_val['trig_callback'] == 'joint_pose_save':
                joint_topic_addr = conf_val['robot_ns'] + conf_val['joint_topic']
                conf_val['subscriber'] = rospy.Subscriber(joint_topic_addr, JointState, callback=self._joint_state_cb, callback_args=(conf_key))
            if conf_val['trig_callback'] == 'joint_dmp_save':
                joint_topic_addr = conf_val['robot_ns'] + conf_val['joint_topic']
                conf_val['subscriber'] = rospy.Subscriber(joint_topic_addr, JointState, callback=self._joint_state_cb, callback_args=(conf_key))
            # Add TF

        # Remove duplicate namespaces
        self._available_ns = list(dict.fromkeys(tmp_ns))

    def _write_conf_to_table(self):
        self._conf_table.setRowCount(len(self._conf_yaml))
        for row, conf_key in enumerate(self._conf_yaml.items()):
            conf = conf_key[1]
            self._conf_table.setItem(row, 0, QTableWidgetItem(conf['trig_name']))
            self._conf_table.setItem(row, 1, QTableWidgetItem(conf['robot_ns']))
            self._conf_table.setItem(row, 2, QTableWidgetItem(conf['trig_topic']))
            self._conf_table.setItem(row, 3, QTableWidgetItem(conf['trig_type']))
            self._conf_table.setItem(row, 4, QTableWidgetItem(str(conf['trig_value'])))
            self._conf_table.setItem(row, 5, QTableWidgetItem(conf['trig_callback']))

    def _update_data_table(self):
        self._data_table.setRowCount(0)

        for data_key, data_dict in self._data_buffer.items():
            current_row = self._data_table.rowCount()
            self._data_table.insertRow(current_row)
            self._data_table.setItem(current_row, 0, QTableWidgetItem(data_dict['time']))
            self._data_table.setItem(current_row, 1, QTableWidgetItem(data_dict['trig_name']))
            self._data_table.setItem(current_row, 2, QTableWidgetItem(data_dict['trig_type']))
            self._data_table.setItem(current_row, 3, QTableWidgetItem(data_dict['data']._type))
            self._data_table.setItem(current_row, 4, QTableWidgetItem(data_key))
        self._data_table.sortItems(0, Qt.DescendingOrder)

    def _display_item_details(self):
        text_box = self._widget.findChild(QTextEdit, 'entry_details')

        try:
            curent_item = self._data_table.currentItem()
            current_row = curent_item.row()
            entry_identifier = self._data_table.item(current_row, 4).text()
            self.status_report['status'] = 'Item selected'
            data_details = self._data_buffer[entry_identifier]['data'],

            text_box.setText(format(data_details))
        except Exception as e:
            pass

    def _display_addresses(self):
        self.discover_available_topics_array()
        addr_list_widget = self._widget.findChild(QListWidget, 'addr_list')

        for topic in self._available_topics_array:
            addr_list_widget.addItem(QListWidgetItem(topic))
        # topic = '/ur10_2/joint_states'
        # self._available_ns

    # Get a list of topics that match a certain type
    def discover_available_topics_array(self):
        all_topics = rospy.get_published_topics()
        for topic, msg_type in all_topics:
            if msg_type in DESIRED_MSGS:
                self._available_topics_array.append(topic)
                self._available_topics_dict[topic] = msg_type

        self._available_topics_array.sort()
        # The following line removes possible duplicate entries from the array
        self._available_topics_array = list(set(self._available_topics_array))

    # Discover available robots
    def discover_frames(self):

        # Define a search pattern, it will look for all namespaces that contain the 'received_control_mode' topic
        search_pattern = re.compile(r'^(?!.*link).*')
        # search_pattern = re.compile(r'^((?!.*link)|(.*base)).*$')

        # Get the list of all currently availabel frames in TF1 (this is a temporary solution until a better one is found)
        time.sleep(1) # Give tf time to start working
        # all_frames = self._tf_listener.getFrameStrings()

        yaml_frames = yaml.load(self._tf2_buffer.all_frames_as_yaml())

        # Look through all avaiable topics
        for entry in yaml_frames:
            namespace = search_pattern.findall(entry)
            if len(namespace):
                self._available_frames.append(namespace[0])

        # Sort the namespaces alphabetically
        self._available_frames.sort()

        # rospy.logdebug out the result of the search
        # rospy.loginfo("Found robots with namespace: \n%s", self._available_frames)

        # Add robots namespaces to combobox
        self._combobox = self._widget.findChildren(QComboBox, QRegExp('frameSelector'))[0]
        self._combobox.insertItems(0, self._available_frames)
        self._namespace = self._combobox.currentText()

        # Quit if no robots are found
        if len(self._available_frames) == 0:
            rospy.logerr("[%s]: No robots available !!", rospy.get_name())
            sys.exit()

    def _save_button_pressed(self, save_button):

        save_name_widget = self._widget.findChild(QLineEdit, 'save_name')
        save_name = save_name_widget.text()

        # Check if a name was entered
        if not(len(save_name)):
            self.status_report['status'] = 'No name provided !!'
            return -1

        # Get selected item
        curent_item = self._data_table.currentItem()
        if curent_item is None:
            self.status_report['status'] = 'No item selected !!'
            return -1

        # Get selected item row
        current_row = curent_item.row()
        if current_row == -1:
            self.status_report['status'] = 'No item selected !!'
            return -1

        # Get the item identifier and retrieve the data
        entry_identifier = self._data_table.item(current_row, 4).text()
        data = self._data_buffer[entry_identifier]['data']

        # Update the status box
        self.status_report['status'] = 'Save button pressed'
        rospy.logdebug("Save button pressed !!")

        # Handle transform stamped
        if data._type == 'geometry_msgs/TransformStamped':
            trig_name = self._data_buffer[entry_identifier]['trig_name']
            for keys, vals in self._data_buffer.items():
                if (vals['trig_name'] == trig_name) and (vals['data']._type == 'sensor_msgs/JointState'):
                    pass

        # Handle Joint space configuration
        elif data._type == 'sensor_msgs/JointState':
            self._joint_capture_srv.call(data, save_name)

        # Handle Joint space configuration
        elif data._type == 'robot_module_msgs/JointSpaceDMP':
            self._dmp_capture_srv.call(data, CartesianSpaceDMP(), save_name)

        # Handle Joint space DMP

        # # Saving to database according to the text in the saveName field
        # if self._joint_space:
        #     existingDMP = self._msg_store.query_named(saveName, JointSpaceDMP._type)[0]
        #     if existingDMP == None:
        #         rospy.loginfo("Saving new DMP with ID = " + self._dmp_save_name + " to database!")
        #         self._msg_store.insert_named(saveName, self._recorded_dmp)
        #     else:
        #         rospy.loginfo("Updating DMP with ID = " + self._dmp_save_name + " in database!")
        #         self._msg_store.update_named(saveName, self._recorded_dmp)
        # else:
        #     existingDMP = self._msg_store.query_named(saveName, CartesianSpaceDMP._type)[0]
        #     if existingDMP == None:
        #         rospy.loginfo("Saving new DMP with ID = " + self._dmp_save_name + " to database!")
        #         self._msg_store.insert_named(saveName, self._recorded_dmp)
        #     else:
        #         rospy.loginfo("Updating DMP with ID = " + self._dmp_save_name + " in database!")
        #         self._msg_store.update_named(saveName, self._recorded_dmp)

        # Write to report
        self.status_report['status'] = 'Data saved'
        self.status_report['last_entry'] = save_name
        self.status_report['last_type'] = data._type

        # Update text fields
        save_name_widget.setText('')
        details_text_box = self._widget.findChild(QTextEdit, 'entry_details')
        details_text_box.setText('')

        # Clean up table
        self._data_table.clearSelection()
        self._data_table.removeRow(current_row)

        # # Remove data from buffer
        # self._data_buffer.pop(entry_identifier)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # Topic subscribers
    def _joint_state_cb(self, data, conf_name):
        self._conf_yaml[conf_name]['joint_data'] = data

    # def save_joint_conf_sig_h(self):
    #     rospy.loginfo('Saving joint conf !!')

    def _rising_edge_cb(self, data, conf_name):

        if not(self._conf_yaml[conf_name].has_key('sig_val')):
            self._conf_yaml[conf_name]['sig_val'] = data.data

        old_val = self._conf_yaml[conf_name]['sig_val']
        new_val = data.data
        if old_val - new_val == -1:
            rospy.logdebug('Rising edge!!')
            self._handle_trigger(self._conf_yaml[conf_name])

        self._conf_yaml[conf_name]['sig_val'] = new_val

    def _falling_edge_cb(self, data, conf_name):

        if not(self._conf_yaml[conf_name].has_key('sig_val')):
            self._conf_yaml[conf_name]['sig_val'] = data.data

        old_val = self._conf_yaml[conf_name]['sig_val']
        new_val = data.data
        if old_val - new_val == 1:
            rospy.logdebug('Falling edge!!')
            self._handle_trigger(self._conf_yaml[conf_name])

        self._conf_yaml[conf_name]['sig_val'] = new_val

    def _hold_cb(self, data, conf_name):
        if not('sig_val' in self._conf_yaml[conf_name]):
            self._conf_yaml[conf_name]['sig_val'] = data.data

        old_val = self._conf_yaml[conf_name]['sig_val']
        new_val = data.data
        if falling_edge(old_val, new_val):
            rospy.logdebug("[_hold_cb] Falling edge")
            if len(self._joint_traj):
                self._handle_trigger(self._conf_yaml[conf_name])

        if rising_edge(old_val, new_val):
            pass
            rospy.logdebug("[_hold_cb] Rising edge")

        if old_val + new_val == 2:
            rospy.logdebug("[_hold_cb] Holding")
            trig_type = self._conf_yaml[conf_name]['trig_callback']
            if trig_type == "joint_dmp_save":
                rospy.logdebug("Appending !!")
                self._joint_traj.append(self._conf_yaml[conf_name]['joint_data'])

        self._conf_yaml[conf_name]['sig_val'] = new_val

    def _handle_trigger(self, trigg_conf):
        trigg_conf['robot_ns']

        trig_type = trigg_conf['trig_callback']

        save_data = []

        if trig_type == "joint_save":
            rospy.logdebug('joint_save')
            save_data = [trigg_conf['joint_data']]

        elif trig_type == "joint_pose_save":
            joint_data = trigg_conf['joint_data']
            rospy.logdebug('joint_pose_save')
            save_data = [joint_data, TransformStamped()]  # , tf_data]

        elif trig_type == "joint_dmp_save":
            rospy.logdebug('joint_dmp_save')

            traj_dur = traj_duration_sec(self._joint_traj)
            if traj_dur < 1:
                rospy.logwarn("Trajectory too short to store")
            else:
                dmp_request = EncodeTrajectoryToDMPRequest()
                dmp_request.demonstratedTrajectory = self._joint_traj
                dmp_request.dmpParameters.N = trigg_conf['joint_dmp_N']
                result_dmp = self._dmp_ecoder.call(dmp_request).encodedDMP
                save_data = [result_dmp]
                self._joint_traj = []

        else:
            rospy.logerr("Unknown saving type !!")
            rospy.logerr("trig_type = {}".format(trig_type))
            # self.status_report['status'] = "Error. See terminal"

        for data in save_data:
            now = datetime.now()
            entry_identifier = now.strftime("%H%M%S%f")
            current_time = now.strftime("%H:%M:%S")
            self._data_buffer[entry_identifier] = {
                'data': data,
                'time': current_time,
                'trig_name': trigg_conf['trig_name'],
                'trig_type': trigg_conf['trig_type']
            }
            self._refresh_data_table_contents_sig.emit()


    # def digitalinput_cb(self, digitalinput):
    #     """Callback method for retreiving robot pose."""
    #     self._digitalinput = digitalinput.data
    #     # rospy.loginfo(self._digitalinput)
    #     # widget = QWidget()
    #     # status_text_widget = widget.findChildren(QTextEdit, QRegExp('status'))[0]
    #     # Start or stop DMP recording
    #     if ((int(self._digitalinput) >> BUTTON_2_PIN) - self._old_rec_button) == 1 and self._gravComp:
    #         rospy.loginfo("Started DMP recording !!")
    #         self._dmp_recording_started_signal.emit()
    #         goal = dmp_record_tool_msgs.msg.RecordDMPGoal([self._topic_name], self._num_weights)
    #         self._dmp_action_server_client.send_goal(goal)
    #         self._recording = True
    #         # self._stream.write(self._beep)


    #     if ((int(self._digitalinput) >> BUTTON_2_PIN) - self._old_rec_button) == -1 and self._recording:
    #         rospy.loginfo("Stopped DMP recording !!")
    #         self._dmp_action_server_client.cancel_goal()
    #         self._dmp_recording_stopped_signal.emit()
    #         self._recording = False
    #         # self._stream.write(self._beep)

    #     # Set the robot into or out of gravity compensation mode
    #     if ((int(self._digitalinput) >> BUTTON_1_PIN) - self._old_gravbutton) == 1:
    #         rospy.loginfo("Gravity compensation is activated.")
    #         #self._gravcomp_service(GRAVCOMP_ON)
    #         self._gravComp = True

    #     if ((int(self._digitalinput) >> BUTTON_1_PIN) - self._old_gravbutton) == -1:
    #         #rospy.loginfo("Gravity compensation is deactivated.")
    #         self._gravcomp_service(GRAVCOMP_OFF)
    #         self._gravComp = False

    #     # Save the current values to use them in the next cycle
    #     self._old_gravbutton = int(self._digitalinput) >> BUTTON_1_PIN
    #     self._old_rec_button = int(self._digitalinput) >> BUTTON_2_PIN
