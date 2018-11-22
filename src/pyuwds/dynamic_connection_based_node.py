#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from threading import Lock
from uwds_msgs.srv import List
from enum import Enum
from types import *

DEFAULT_PUBLISHER_BUFFER_SIZE = 10
DEFAULT_SUBSCRIBER_BUFFER_SIZE = 10
DEFAULT_TIME_SYNCHRONIZER_BUFFER_SIZE = 10



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
        self.connection_status = NOT_INITIALIZED
        self.input_worlds = []
        self.ouput_worlds = []
        self.node_name = node_name
        self.verbose = rospy.get_param("~verbose", True)
        self.publiser_buffer_size = rospy.get_param("~publisher_buffer_size",
                        DEFAULT_PUBLISHER_BUFFER_SIZE)
        self.publiser_buffer_size = rospy.get_param("~subscriber_buffer_size",
                        DEFAULT_SUBSCRIBER_BUFFER_SIZE)

        self.time_synchronizer_buffer_size = rospy.get_param("~time_synchronizer_buffer_size",
                        DEFAULT_TIME_SYNCHRONIZER_BUFFER_SIZE)

        self.list_input_worlds_server = rospy.Service("~input_worlds",
                                                      List,
                                                      self.listInputWorlds)
        self.list_output_worlds_server = rospy.Service("~output_worlds",
                                                       List,
                                                       self.listOutputWorlds)
        self.connection_mutex = Lock()
        self.input_worlds = []
        self.output_worlds = []
        self.changes_publisher_map = {}
        self.changes_subscriber_map = {}
        self.sync_changes_subscribers_map = {}

    def resetInputWorlds(self):
        """
        """
        self.input_worlds = []

    def resetOutputWorlds(self):
        """
        """
        self.ouput_worlds = []

    def addInputWorld(self, world):
        """
        """
        if world not in self.input_worlds:
            self.input_worlds.append(world)

    def addOutputWorld(self, world):
        """
        """
        if world not in self.ouput_worlds:
            self.ouput_worlds.append(world)

    def listInputWorlds(self, req):
        """
        """
        try:
            rospy.logdebug("[%s] Service '~input_worlds' requested",
                           self.node_name)
            if self.connection_status == NOT_INITIALIZED:
                return [], False, "Node not initialized, try later !"
            if self.connection_status == NOT_CONNECTED:
                return [], False, "Node not connected, try to reconfigure it !"
            if self.connection_status == CONNECTING:
                return [], False, "Node currently connecting, try later !"
            return self.input_worlds, True, ""
        except Exception as e:
            rospy.logerr("[%s] Error occured when calling '~input_worlds'",
                         self.node_name)

    def listOutputWorlds(self, req):
        """
        """
        try:
            rospy.logdebug("[%s] Service '~output_worlds' requested",
                           self.node_name)
            if self.connection_status == NOT_INITIALIZED:
                return [], False, "Node not initialized, try later !"
            if self.connection_status == NOT_CONNECTED:
                return [], False, "Node not connected, try to reconfigure it !"
            if self.connection_status == CONNECTING:
                return [], False, "Node currently connecting, try later !"
            return self.output_worlds, True, ""
        except Exception as e:
            rospy.logerr("[%s] Error occured when calling '~output_worlds'",
                         self.node_name)
