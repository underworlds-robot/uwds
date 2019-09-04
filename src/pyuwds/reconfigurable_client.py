#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Header
from uwds_msgs.msg import Invalidations
from uwds_msgs.srv import ReconfigureInputs, List
from pyuwds.uwds_client import UwdsClient


class ReconfigurableClient(UwdsClient):
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
        super(ReconfigurableClient, self).__init__(client_name, client_type)
        input_worlds = rospy.get_param("~default_inputs", "")
        self.__use_single_input = rospy.get_param("~use_single_input", False)
        self.input_worlds = input_worlds.split(" ")
        self.reconfigure(input_worlds.split(" "))
        self.__reconfigure_service_server = rospy.Service(client_name+"/reconfigure_inputs", ReconfigureInputs, self.reconfigureInputs)
        self.__list_inputs_service_server = rospy.Service(client_name+"/list_inputs", List, self.listInputs)
        rospy.loginfo("[{}::init] Underworlds client ready !".format(self.ctx.name()))

    def reconfigure(self, inputs):
        if len(inputs) > 1 and self.__use_single_input:
            raise RuntimeError("Multiple inputs provided while 'use_single_input' activated.")
        self.ctx.worlds().close()
        self.onReconfigure(inputs)
        for input in inputs:
            self.ctx.worlds()[input].connect(self.onChanges)
            invalidations = Invalidations()
            scene = self.ctx.worlds()[input].scene()
            timeline = self.ctx.worlds()[input].timeline()
            meshes = self.ctx.worlds()[input].meshes()
            for node in scene.nodes():
                invalidations.node_ids_updated.append(node.id)
            for situation in timeline.situations():
                invalidations.situation_ids_updated.append(situation.id)
            for mesh in meshes:
                invalidations.mesh_ids_updated.append(mesh.id)
            header = Header()
            header.stamp = rospy.Time.now()
            self.onChanges(input, header, invalidations)
        self.inputs_worlds = inputs

    def reconfigureInputs(self, req):
        try:
            self.reconfigure(req.inputs)
            return True, ""
        except Exception as e:
            return False, str(e)

    def listInputs(self, req):
        try:
            return self.input_worlds, True, ""
        except Exception as e:
            return [], False, str(e)

    def onChanges(self, world_name, header, invalidations):
        raise NotImplementedError

    def onReconfigure(self, input_worlds):
        raise NotImplementedError
