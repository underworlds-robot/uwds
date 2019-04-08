#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from pyuwds.uwds import UnderworldsProxy

class UwdsClient(object):
    """
    The Underworlds client
    """
    def __init__(self, client_name, client_type):
        """
        The Underworlds client

        @type self.client_name: string
        @param self.client_name: The client name
        @type self.client_type: string
        @param self.client_type: The client type
        """
        self.verbose = rospy.get_param("~verbose", True)
        self.global_frame_id = rospy.get_param("~global_frame_id", "map")
        self.output_world = rospy.get_param("~output_world", "")
        self.output_suffix = rospy.get_param("~output_suffix", "")
        self.ctx = UnderworldsProxy(client_name, client_type)
