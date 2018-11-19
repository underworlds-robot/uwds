#!/usr/bin/env python
# -*- coding: utf-8 -*-
import uuid
from uwds_msgs.msg import *
from uwds_msgs.srv import *


class NodeType(Enum):
    UNDEFINED = 0
    ENTITY = 1
    MESH = 2
    CAMERA = 3


NodeTypeNames = {0: "undefined", 1: "entity", 2: "mesh", 3: "camera"}


class SituationType(Enum):
    GENERIC = 0
    FACT = 1
    ACTION = 2
    INTERNAL = 3


NodeTypeNames = {0: "generic", 1: "fact", 2: "action", 3: "internal"}


class ClientInteractionType(Enum):
    READ = 0
    WRITE = 1


ClientInteractionTypeNames = {0: "read", 1: "write"}


class ClientType(Enum):
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
        self.root_id = str(uuid.uuid4())
        root_node = Node()
        root_node.id = self.root_id
        root_node.name = "root"
        root_node.position.pose.orientation.w = 1.0
        self.nodes = {}
        self.nodes[self.root_id] = root_node


    def update(nodes):
        for node in nodes:
            self.nodes[node.id] = node

    def remove(node_ids):
        for node_id in nodes:
            del self.nodes[node_id]

    def reset(self, root_id):
        self.root_id = root_id
        root_node = Node()
        root_node.id = self.root_id
        root_node.name = "root"
        root_node.position.pose.orientation.w = 1.0
        self.nodes = {}
        self.nodes[self.root_id] = root_node

    def reset(self):
        self.root_id = str(uuid.uuid4())
        root_node = Node()
        root_node.id = self.root_id
        root_node.name = "root"
        root_node.position.pose.orientation.w = 1.0
        self.nodes = {}
        self.nodes[self.root_id] = root_node

    def getWorldPose(node_id):
        # TODO:
        raise NotImplementedError

    def lookUpPose(source_node_id, target_node_id):
        # TODO:
        raise NotImplementedError


class Timeline(object):
    """
    The Underworlds timeline data structure
    """
    def __init__(self):
        self.origin = rospy.Time.now()
        self.situations = {}

    def reset(self, origin):
        self.origin = origin
        self.situations = {}

    def reset(self):
        self.origin = rospy.Time.now()
        self.situations = {}


class World(object):
    """
    The Underworlds world data structure
    """
    def __init__(self, name, meshes):
        self.scene = Scene()
        self.timeline = Timeline()
        self.meshes = meshes
        self.name = name

    def applyChanges(header, changes):
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
        self.meshes.update(changes.meshes_to_update)
        for mesh in changes.meshes_to_update:
            invalidations.mesh_ids_updated.append(mesh.id)
        return invalidations

    def reset():
        self.scene.reset()
        self.timeline.reset()


class Topology(object):
    """
    The Underworlds client topology structure
    """
    def __init__(self):
        self.clients = {}
        self.client_types = {}
        self.client_interactions = {}

    def update(ctxt, interaction_type):


    def reset(worlds, clients, client_interactions):
        pass


class UwdsBase(object):
    """The Underworlds base data structure
    """
    def __init__(self):
        self.worlds = Worlds()
        self.meshes = {}
        self.topology = Topology()
