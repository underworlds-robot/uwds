#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from pyuwds.uwds import UnderworldsProxy

class ReconfigurableClient(object):
    """
    The Underworlds client

    @type self.node_name: string
    @param self.node_name: The client name
    """
    def __init__(self, client_name, client_type):
        """
        The Underworlds client

        @type self.node_name: string
        @param self.node_name: The client name
        """

        self.__use_single_input = rospy.get_param("~use_single_input", False)
        self.input_worlds = rospy.get_param("~default_input_worlds", "")
        self.output_suffix = rospy.get_param("~output_suffix", "")
        self.ctx = UnderworldsProxy(client_name, client_type)
        self.__reconfigure_service_server = rospy.Service(client_name+"/reconfigure_inputs", self.reconfigureInputs)

    def reconfigure(inputs):
        pass

    def reconfigureInputs(req):
        return True

    def onChanges(world_name, header, invalidations):
        pass


    def onReconfigure(input_worlds):
        pass
