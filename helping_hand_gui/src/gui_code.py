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
from python_qt_binding.QtWidgets import QTextEdit
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
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from std_srvs.srv import Empty
from robot_module_msgs.msg import JointSpaceDMP
from robot_module_msgs.msg import CartesianSpaceDMP
import actionlib

# YAML
import yaml

# TF
import tf2_ros
import tf

import time
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


# A thread for the monitoring function
# import thread

class DMPRecordToolGUI(Plugin):

    # Various member variables
    _topic_subscriber = None
    _topic_good = False
    _digitalinput_subscriber = None
    _namespace = None
    _pattern = re.compile(r'\/[^/]+\/')
    _digitalinput = None
    _gravcomp_service = None
    _dmp_action_server_client = None
    _topic_name = None
    _num_weights = 0
    _recorded_dmp = None
    _msg_store = None
    _database_service = None
    _dmp_save_name = None
    _pyaudio = None
    _stream = None
    _beep = None
    _joint_space = None
    _cartesian_space = None
    _available_frames = []

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
        super(DMPRecordToolGUI, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('DMPRecordToolGUI')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

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

        # Populate table's headers
        self._conf_table = self._widget.findChildren(QTableWidget, QRegExp('configTable'))[0]
        self._conf_table.setHorizontalHeaderLabels(('Trigger Name','Trigger Topic', 'Trigger Type', 'Trigger Value', 'Trigger Callback'))
        
        # Parse config
        self._parse_yaml_config()


        # available_topics = ['ur10_2/digital_outputs', 'ur10_1/digital_outputs', 'ur10_2/button_1']
        # current_row = 0
        # combo = QComboBox('conf_topic_{}'.format(current_row))
        # for t in available_topics:
        #         combo.addItem(t)
        # conf_table.setCellWidget(current_row,1,combo)
        
        # RECONCELL CODE

        # Discover all available frames
        # self.discover_frames()

        # Route signals from widgets to callback functions

        # Save button widget
        # save_button = self._widget.findChildren(QPushButton, QRegExp('databaseSaveButton'))[0]
        # save_button.clicked.connect(partial(self._save_button_pressed, save_button))

        # Topic name change signal handler
        # save_name_widget = self._widget.findChildren(QLineEdit, QRegExp('databaseSaveName'))[0]
        # save_name_widget.textChanged.connect(partial(self._save_name_changed,
        # save_name_widget))
        # Combobox for available topics
        # self._topics_combobox = self._widget.findChildren(QComboBox, QRegExp('topicSelector'))[0]
        # self._topics_combobox.currentIndexChanged.connect(partial(self._topic_select_cb))
        # self._topics_combobox.highlighted.connect(partial(self._topic_info_cb))

        # Weight change signal handler
        # num_weights_widget = self._widget.findChildren(QLineEdit, QRegExp('numWeights'))[0]
        # self._num_weights = int(num_weights_widget.text())
        # num_weights_widget.textChanged.connect(partial(self._num_weights_set, num_weights_widget))

        # Handle dmp stop and start signals
        # self._dmp_recording_started_signal.connect(self.dmp_started_signal_handler)
        # self._dmp_recording_stopped_signal.connect(self.dmp_stopped_signal_handler)

        # Combobox for tf frames widget
        # namespace_combo_box = self._widget.findChildren(QComboBox, QRegExp('frameSelector'))[0]
        # namespace_combo_box.currentIndexChanged.connect(partial(self._frame_combobox))

        # Create database proxy
        self._msg_store = MessageStoreProxy()

        # Create action server client
        # self._dmp_action_server_client = actionlib.SimpleActionClient('dmp_record_action_server', dmp_record_tool_msgs.msg.RecordDMPAction)
        # if self._dmp_action_server_client.wait_for_server(rospy.Duration.from_sec(1)) == False:
        #     rospy.logerr('Could not initialize' + rospy.get_name() + ' !!')
        #     sys.exit('DMP Action Server not found!')

        # Create database reload service
        # try:
        #     rospy.wait_for_service('/apps/database/reload', 0.1)
        #     self._database_service = rospy.ServiceProxy('/apps/database/reload', Empty)
        # except:
        #     rospy.logerr('Could not initialize' + rospy.get_name() + ':')
        #     sys.exit('Database reload service is unreachable !!')

        # Initialize the available topics variable
        # self.discover_available_topics_array()

        # Initialize values of the combo box
        # self._topics_combobox.insertItems(0, self._available_topics_array)
        # self._topics_combobox.setCurrentIndex(0)
        # self._topics_combobox.setToolTip('Select a topic to subscribe to.')

        # # Create audio variables and objects
        # self._pyaudio = pyaudio.PyAudio()
        #
        # volume = 0.75  # range [0.0, 1.0]
        # fs = 44100  # sampling rate, Hz, must be integer
        # duration = 1.0  # in seconds, may be float
        # f = 440.0  # sine frequency, Hz, may be float
        #
        # # generate samples, note conversion to float32 array
        # samples = (np.sin(2 * np.pi * np.arange(fs * duration) * f / fs)).astype(np.float32)
        #
        # # for paFloat32 sample values must be in range [-1.0, 1.0]
        # self._stream = self._pyaudio.open(format=pyaudio.paFloat32,
        #                 channels=1,
        #                 rate=fs,
        #                 output=True)
        #
        # self._beep = volume*samples

    # YAML config parser
    def _parse_yaml_config(self):
        conf_file = os.path.join(rospkg.RosPack().get_path('helping_hand_gui'), 'conf', 'triggers_conf.yml')
        conf_yaml = yaml.load(open(conf_file), Loader=yaml.FullLoader)
        self._conf_table.setRowCount(len(conf_yaml))
        # row = 0
        for row, conf_key in enumerate(conf_yaml.items()):
            conf = conf_key[1]
            self._conf_table.setItem(row, 0, QTableWidgetItem(conf['trig_name']))
            self._conf_table.setItem(row, 1, QTableWidgetItem(conf['trig_topic']))
            self._conf_table.setItem(row, 2, QTableWidgetItem(conf['trig_type']))
            self._conf_table.setItem(row, 3, QTableWidgetItem(conf['trig_value']))
            self._conf_table.setItem(row, 4, QTableWidgetItem(conf['trig_callback']))

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

        # Print out the result of the search
        # rospy.loginfo("Found robots with namespace: \n%s", self._available_frames)

        # Add robots namespaces to combobox
        self._combobox = self._widget.findChildren(QComboBox, QRegExp('frameSelector'))[0]
        self._combobox.insertItems(0, self._available_frames)
        self._namespace = self._combobox.currentText()

        # Quit if no robots are found
        if len(self._available_frames) == 0:
            rospy.logerr("[%s]: No robots available !!", rospy.get_name())
            sys.exit()

    def dmp_started_signal_handler(self):
        status_text_widget = self._widget.findChildren(QTextEdit, QRegExp('status'))[0]
        status_text_widget.setText("Recording")
        status_text_widget.setStyleSheet('QTextEdit {color: red}')

    def dmp_stopped_signal_handler(self):
        # After the DMP has been recorded, wait for the action server to tell you that
        self._dmp_action_server_client.wait_for_result()

        # Get the result
        recorded_dmps = self._dmp_action_server_client.get_result()
        self._recorded_dmp = recorded_dmps.JointDMP if self._joint_space else recorded_dmps.CartesianDMP
        self._dmp_save_name = str(self._recorded_dmp.id)

        # Display the DMP ID in the save field
        save_name_widget = self._widget.findChildren(QLineEdit, QRegExp('databaseSaveName'))[0]
        save_name_widget.setText(self._dmp_save_name)

        status_text_widget = self._widget.findChildren(QTextEdit, QRegExp('status'))[0]
        status_text_widget.setText("Idle")
        status_text_widget.setStyleSheet('QTextEdit {color: black}')

    def _topic_select_cb(self):

        self._topic_name = self._topics_combobox.currentText()

        # Adjust the topic info field
        self.set_topic_info_text(self._topic_name)

        # Get the topic msg type
        selected_topic_type = rostopic.get_topic_class(self._topic_name)[0]
        # Define in which space we are recording the DMP using in other parts of the code
        self._joint_space = True if (selected_topic_type == JointState) else False

        self._topic_good = True
        # Extract namespace
        self._namespace = self._pattern.findall(self._topic_name)[0]

        # Make sure we have a clean start everytime the topic changes
        if self._digitalinput_subscriber is not None:
            self._digitalinput_subscriber.unregister()
            self._digitalinput_subscriber = None
        if self._gravcomp_service is not None:
            self._gravcomp_service.close()
            self._gravcomp_service = None

        # Subscribe the callback for digital input
        self._digitalinput_subscriber = rospy.Subscriber(self._namespace + 'digital_inputs', Float64,
                                                            self.digitalinput_cb)
        # Register the gravity compensation service proxy
        try:
            rospy.wait_for_service(self._namespace + 'gravity_compensation_switch', 0.2)
            self._gravcomp_service = rospy.ServiceProxy(self._namespace + 'gravity_compensation_switch', SetBool)
        except Exception as e:
            rospy.logerr(
                'Selected namespace [{0}] does not provide the neccesarry '
                'services to properly utilize this tool.'.format(
                   self._topic_name
                ))
            rospy.logerr('Selection was reverted to the previous one.')
            self._topics_combobox.setCurrentIndex(self._old_index)
        self._old_index = self._topics_combobox.currentIndex()

    def set_topic_info_text(self, topic):
        topic_info_field = self._widget.findChildren(QTextEdit, QRegExp('topicInfo'))[0]
        topic_info_field.setText(self._available_topics_dict[topic])


    def _topic_info_cb(self, highlighted_topic_index):
        highlighted_topic = self._topics_combobox.itemText(highlighted_topic_index)
        highlighted_topic_type = rostopic.get_topic_type(highlighted_topic)[0]
        info_string = '{0}'.format(highlighted_topic_type)
        self._topics_combobox.setItemData(
            highlighted_topic_index,
            info_string,
            Qt.ToolTipRole)

    # def _topic_name_changed(self, topic_text_widget):

    #     topicName = topic_text_widget.text()
    #     self._topic_name = topicName

    #     # Get the topic msg type
    #     topicMsgType = rostopic.get_topic_class(topicName)[0]

    #     # Perform message type check and subscribe callbacks to topics and services
    #     if (topicMsgType == JointState) or (topicMsgType == PoseStamped):

    #         # Define in which space we are recording the DMP using in other parts of the code
    #         self._joint_space = True if (topicMsgType == JointState) else False

    #         self._topic_good = True
    #         # Extract namespace
    #         self._namespace = self._pattern.findall(topicName)[0]

    #         # Subscribe the callback for digital input
    #         self._digitalinput_subscriber = rospy.Subscriber(self._namespace + 'digital_inputs', Float64,
    #                                                          self.digitalinput_cb)
    #         # Register the gravity compensation service proxy
    #         try:
    #             rospy.wait_for_service(self._namespace + 'gravity_compensation_switch', 0.2)
    #             self._gravcomp_service = rospy.ServiceProxy(self._namespace + 'gravity_compensation_switch', SetBool)
    #         except:
    #             rospy.logerr('Could not initialize' + rospy.get_name() + ':')
    #             sys.exit('Gravity compensation services is not reachable !!')

    #         # Change the text color to green
    #         topic_text_widget.setStyleSheet('QLineEdit {color: green}')
    #     else:
    #         self._topic_good = False
    #         # Unsubscribe from topics
    #         if self._digitalinput_subscriber != None:
    #             self._digitalinput_subscriber.unregister()
    #             self._digitalinput_subscriber = None
    #             self._gravcomp_service.close()
    #             self._gravcomp_service = None

    #         topic_text_widget.setStyleSheet('QLineEdit {color: red}')

    def _save_name_changed(self, save_name_widget):
        self._dmp_save_name = save_name_widget.text()
        # rospy.loginfo(self._dmp_save_name)

    def _num_weights_set(self, num_weights_widget):
        num_weights_text = num_weights_widget.text()
        if num_weights_text.isdigit():
            self._num_weights = int(num_weights_widget.text())
            num_weights_widget.setStyleSheet('QLineEdit {color: black}')
        else:
            num_weights_widget.setStyleSheet('QLineEdit {color: red}')

    def _register_robot_widget_handlers(self, gui_widgets):
        rospy.loginfo("_register_robot_widget_handlers")
        save_button = gui_widgets.findChildren(QPushButton, QRegExp('databaseSaveButton'))
        save_button.mousePressEvent.connect(self._save_button_pressed, save_button)
        save_button.setEnabled(False)
        rospy.loginfo(save_button)

    def _save_button_pressed(self, save_button):

        save_name_widget = self._widget.findChildren(QLineEdit, QRegExp('databaseSaveName'))[0]
        saveName = save_name_widget.text()

        print self._recorded_dmp.w
        print type(self._recorded_dmp)

        # Saving to database according to the text in the saveName field
        if self._joint_space:
            existingDMP = self._msg_store.query_named(saveName, JointSpaceDMP._type)[0]
            if existingDMP == None:
                rospy.loginfo("Saving new DMP with ID = " + self._dmp_save_name + " to database!")
                self._msg_store.insert_named(saveName, self._recorded_dmp)
            else:
                rospy.loginfo("Updating DMP with ID = " + self._dmp_save_name + " in database!")
                self._msg_store.update_named(saveName, self._recorded_dmp)
        else:
            existingDMP = self._msg_store.query_named(saveName, CartesianSpaceDMP._type)[0]
            if existingDMP == None:
                rospy.loginfo("Saving new DMP with ID = " + self._dmp_save_name + " to database!")
                self._msg_store.insert_named(saveName, self._recorded_dmp)
            else:
                rospy.loginfo("Updating DMP with ID = " + self._dmp_save_name + " in database!")
                self._msg_store.update_named(saveName, self._recorded_dmp)


        self._database_service()

        # Print this DMP id
        last_saved_name_widget = self._widget.findChildren(QTextEdit, QRegExp('lastSavedName'))[0]
        last_saved_name_widget.setText(saveName)

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

        #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    ## Topic subscribers
    def topic_cb(self, data):
        """Callback method for retreiving robot joints."""
        rospy.loginfo(data)

    def digitalinput_cb(self, digitalinput):
        """Callback method for retreiving robot pose."""
        self._digitalinput = digitalinput.data
        # rospy.loginfo(self._digitalinput)
        # widget = QWidget()
        # status_text_widget = widget.findChildren(QTextEdit, QRegExp('status'))[0]
        # Start or stop DMP recording
        if ((int(self._digitalinput) >> BUTTON_2_PIN) - self._old_rec_button) == 1 and self._gravComp:
            rospy.loginfo("Started DMP recording !!")
            self._dmp_recording_started_signal.emit()
            goal = dmp_record_tool_msgs.msg.RecordDMPGoal([self._topic_name], self._num_weights)
            self._dmp_action_server_client.send_goal(goal)
            self._recording = True
            # self._stream.write(self._beep)


        if ((int(self._digitalinput) >> BUTTON_2_PIN) - self._old_rec_button) == -1 and self._recording:
            rospy.loginfo("Stopped DMP recording !!")
            self._dmp_action_server_client.cancel_goal()
            self._dmp_recording_stopped_signal.emit()
            self._recording = False
            # self._stream.write(self._beep)

        # Set the robot into or out of gravity compensation mode
        if ((int(self._digitalinput) >> BUTTON_1_PIN) - self._old_gravbutton) == 1:
            rospy.loginfo("Gravity compensation is activated.")
            #self._gravcomp_service(GRAVCOMP_ON)
            self._gravComp = True

        if ((int(self._digitalinput) >> BUTTON_1_PIN) - self._old_gravbutton) == -1:
            #rospy.loginfo("Gravity compensation is deactivated.")
            self._gravcomp_service(GRAVCOMP_OFF)
            self._gravComp = False

        # Save the current values to use them in the next cycle
        self._old_gravbutton = int(self._digitalinput) >> BUTTON_1_PIN
        self._old_rec_button = int(self._digitalinput) >> BUTTON_2_PIN
