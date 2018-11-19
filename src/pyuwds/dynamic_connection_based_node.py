#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from threading import Lock
from uwds_msgs.srv import List

DEFAULT_PUBLISHER_BUFFER_SIZE = 10
DEFAULT_SUBSCRIBER_BUFFER_SIZE = 10
DEFAULT_TIME_SYNCHRONIZER_BUFFER_SIZE = 10

class ConnectionStatus(Enum):
    NOT_INITIALIZED = 0
    NOT_CONNECTED = 1
    CONNECTING = 2
    CONNECTED = 3

class DynamicConnectionBasedNode(UwdsBase):
    """
    """

    def __init__(self, node_name):
        """
        """
        UwdsBase.__init__(self)
        self.connection_status = ConnectionStatus.NOT_INITIALIZED
        self.input_worlds = []
        self.ouput_worlds = []
        self.node_name = node_name
        rospy.get_param("~verbose", self.verbose, True)
        rospy.get_param("~publisher_buffer_size", self.publiser_buffer_size, DEFAULT_PUBLISHER_BUFFER_SIZE)
        rospy.get_param("~subscriber_buffer_size", self.publiser_buffer_size, DEFAULT_SUBSCRIBER_BUFFER_SIZE)
        rospy.get_param("~time_synchronizer_buffer_size", self.time_synchronizer_buffer_size, DEFAULT_TIME_SYNCHRONIZER_BUFFER_SIZE)

        self.list_input_worlds_server = rospy.Service("~input_worlds", List, self.listInputWorlds)
        self.list_output_worlds_server = rospy.Service("~output_worlds", List, self.listOutputWorlds)

        self.connection_mutex = Lock()

        self.input_worlds = []
        self.output_worlds = []

        self.changes_publisher_map = {}
        self.changes_subscriber_map = {}

        self.sync_changes_subscribers_map = {}

    def resetInputWorldsList(self):
        """
        """
        self.input_worlds = []

    def resetOutputWorldsList(self):
        """
        """
        self.ouput_worlds = []

    def addInputWorld(self, world):
        """
        """
        if connection not in self.input_worlds:
            self.input_worlds.append(connection)

    def addOutputWorld(self, world):
        """
        """
        if connection not in self.ouput_worlds:
            self.ouput_worlds.append(connection)

    def listInputWorlds(self, req):
        """
        """
        try:
            rospy.logdebug("[%s] Service '~input_worlds' requested", self.node_name)
            if connection_status == ConnectionStatus.NOT_INITIALIZED:
                return [], False, "Node not initialized, try later !"
            if connection_status == ConnectionStatus.NOT_CONNECTED:
                return [], False, "Node not connected, try to reconfigure it !"
            if connection_status == ConnectionStatus.CONNECTING:
                return [], False, "Node currently connecting, try later !"
            return input_worlds, True, ""
        except Exception as e:
            rospy.logerr("[%s] Error occured when calling '~input_worlds'", self.node_name)

    def listOutputWorld(req):
        """
        """
        try:
            rospy.logdebug("[%s] Service '~output_worlds' requested", self.node_name)
            if connection_status == ConnectionStatus.NOT_INITIALIZED:
                return [], False, "Node not initialized, try later !"
            if connection_status == ConnectionStatus.NOT_CONNECTED:
                return [], False, "Node not connected, try to reconfigure it !"
            if connection_status == ConnectionStatus.CONNECTING:
                return [], False, "Node currently connecting, try later !"
            return output_worlds, True, ""
        except Exception as e:
            rospy.logerr("[%s] Error occured when calling '~output_worlds'", self.node_name)
