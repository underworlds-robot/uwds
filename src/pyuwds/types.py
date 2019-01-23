#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import uuid
from uwds_msgs.msg import Node, Situation, ClientInteraction, Client, Invalidations

ENTITY = Node.ENTITY
MESH = Node.MESH
CAMERA = Node.CAMERA

NodeTypeNames = {0: "entity", 1: "mesh", 2: "camera"}

GENERIC = Situation.GENERIC
FACT = Situation.FACT
ACTION = Situation.ACTION
INTERNAL = Situation.INTERNAL

NodeTypeNames = {GENERIC: "generic",
                 FACT: "fact",
                 ACTION: "action",
                 INTERNAL: "internal"}

READ = ClientInteraction.READ
WRITE = ClientInteraction.WRITE

ClientInteractionTypeNames = {READ: "read", WRITE: "write"}

UNDEFINED = Client.UNDEFINED
READER = Client.READER
MONITOR = Client.MONITOR
PROVIDER = Client.PROVIDER
FILTER = Client.FILTER

ClientTypeNames = {UNDEFINED: "undefined",
                   READER: "reader",
                   MONITOR: "monitor",
                   PROVIDER: "provider",
                   FILTER: "filter"}


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

    def update(self, nodes):
        node_ids = []
        for node in nodes:
            self.nodes[node.id] = node
            if node.name !="root":
                node_ids.append(node.id)
        return node_ids

    def remove(self, node_ids):
        for node_id in node_ids:
            if node_id in self.nodes:
                del self.nodes[node_id]
        return node_ids

    def reset(self, rootID):
        node_ids = []
        for node_id in self.nodes.keys():
            node_ids.append(node_id)
        self.rootID = rootID
        root_node = Node()
        root_node.id = self.rootID
        root_node.name = "root"
        root_node.position.pose.orientation.w = 1.0
        self.nodes = {}
        self.nodes[self.rootID] = root_node
        return node_id

    def getWorldPose(self, node_id):
        # TODO:
        raise NotImplementedError

    def getWorldPoseWithCovariance(self, node_id):
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
        situation_ids = []
        for situation in situations:
            situation_ids.append(situation.id)
            self.situations[situation.id] = situation
        return situation_ids

    def remove(self, situation_ids):
        for situation_id in situation_ids:
            if situation_id in self.situations:
                del self.situations[situation_id]
        return situation_ids

    def reset(self, origin):
        situation_ids = []
        for situation_id in self.situations.keys():
            node_ids.append(situation_id)
        self.origin = origin
        self.situations = {}
        return situation_ids

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

        invalidations.node_ids_deleted = self.scene.remove(changes.nodes_to_delete)
        invalidations.node_ids_updated = self.scene.update(changes.nodes_to_update)

        invalidations.situation_ids_deleted = self.timeline.remove(changes.situations_to_delete)
        invalidations.situation_ids_updated = self.timeline.update(changes.situations_to_update)

        for mesh_id in changes.meshes_to_delete:
            del self.meshes[mesh_id]
            invalidations.mesh_ids_deleted.append(mesh_id)
        for mesh in changes.meshes_to_update:
            self.meshes[mesh.id] = mesh
            invalidations.mesh_ids_updated.append(mesh.id)
        return invalidations

    def reset(self):
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
