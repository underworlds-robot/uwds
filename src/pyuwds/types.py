#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pyuwds is a light version of Underworlds without
import rospy
import uuid
from uwds_msgs.msg import *
from uwds_msgs.srv import *
from enum import Enum


UNDEFINED = 0
ENTITY = 1
MESH = 2
CAMERA = 3

NodeTypeNames = {0: "undefined", 1: "entity", 2: "mesh", 3: "camera"}


GENERIC = 0
FACT = 1
ACTION = 2
INTERNAL = 3

NodeTypeNames = {0: "generic", 1: "fact", 2: "action", 3: "internal"}


READ = 0
WRITE = 1

ClientInteractionTypeNames = {0: "read", 1: "write"}


UNDEFINED = 0
READER = 1
MONITOR = 2
PROVIDER = 3
FILTER = 4

ClientTypeNames = {0: "reader", 1: "provider", 2: "filter"}


class Scene(object):
    """
    The Underworlds scene data structure
    """
    def __init__(self):
        self.rootID = str(uuid.uuid4())
        root_node = Node()
        root_node.id = self.rootID
        root_node.name = "root"
        root_node.position.pose.orientation.w = 1.0
        self.nodes = {}
        self.nodes[self.rootID] = root_node

    def update(nodes):
        for node in nodes:
            self.nodes[node.id] = node

    def remove(node_ids):
        for node_id in nodes:
            del self.nodes[node_id]

    def reset(self, rootID):
        self.rootID = rootID
        root_node = Node()
        root_node.id = self.rootID
        root_node.name = "root"
        root_node.position.pose.orientation.w = 1.0
        self.nodes = {}
        self.nodes[self.rootID] = root_node

    # def reset(self):
    #     self.rootID = str(uuid.uuid4())
    #     root_node = Node()
    #     root_node.id = self.rootID
    #     root_node.name = "root"
    #     root_node.position.pose.orientation.w = 1.0
    #     self.nodes = {}
    #     self.nodes[self.rootID] = root_node

    def getWorldPose(node_id):
        # TODO:
        raise NotImplementedError

    def getWorldPoseWithCovariance(node_id):
        # TODO:
        raise NotImplementedError

    def getNodeProperty(self, node_id, property_name):
        if node_id in self.nodes:
            for property in self.nodes[node_id].properties:
                if property.name == property_name:
                    return property.data
        return ""


class Timeline(object):
    """
    The Underworlds timeline data structure
    """
    def __init__(self):
        self.origin = rospy.Time.now()
        self.situations = {}

    def update(self, situations):
        for situation in situations:
            self.situations[situation.id] = situation

    def reset(self, origin):
        self.origin = origin
        self.situations = {}

    # def reset(self):
    #     self.origin = rospy.Time.now()
    #     self.situations = {}

    def getSituationProperty(self, situation_id, property_name):
        if situation_id in self.situations:
            for property in self.situations[situation_id].properties:
                if property.name == property_name:
                    return property.data
        return ""


class World(object):
    """
    The Underworlds world data structure
    """
    def __init__(self, name, meshes):
        self.scene = Scene()
        self.timeline = Timeline()
        self.meshes = meshes
        self.name = name

    def applyChanges(self, header, changes):
        invalidations = Invalidations()

        invalidations.node_ids_deleted = changes.nodes_to_delete
        self.scene.update(changes.nodes_to_update)
        for node in changes.nodes_to_update:
            invalidations.node_ids_updated.append(node.id)

        invalidations.situation_ids_deleted = changes.situations_to_delete
        self.timeline.update(changes.situations_to_update)
        for situation in changes.situations_to_update:
            invalidations.situation_ids_updated.append(situation.id)

        invalidations.mesh_ids_deleted = changes.meshes_to_delete
        for mesh in changes.meshes_to_update:
            self.meshes[mesh.id] = mesh
            invalidations.mesh_ids_updated.append(mesh.id)
        return invalidations

    def reset():
        self.scene.reset()
        self.timeline.reset()

class Worlds(object):
    """
    """
    def __init__(self, meshes):
        self.meshes = meshes
        self.worlds = {}

class Topology(object):
    """
    The Underworlds client topology structure
    """
    def __init__(self):
        """
        """
        self.worlds = []
        self.clients = {}
        self.client_interactions = {}

    def update(self, ctxt, interaction_type):
        """
        """
        if ctxt.world not in self.worlds:
            self.worlds.append(ctxt.world)
        if ctxt.client.id not in self.clients:
            self.clients[ctxt.client.id] = ctxt.client
        if ctxt.client.id not in self.client_interactions:
            self.client_interactions[ctxt.client.id] = {}
        #TODO: finish

    def reset(self, worlds, clients, client_interactions):
        """
        """
        self.worlds = worlds
        self.clients = clients
        self.client_interactions = client_interactions


class UwdsBase(object):
    """The Underworlds base data structure
    """
    def __init__(self):
        """
        """
        self.worlds = {}
        self.meshes = {}
        self.topology = Topology()
