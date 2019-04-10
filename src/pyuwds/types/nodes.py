#!/usr/bin/env python
# -*- coding: utf-8 -*-
from concurrent_container import ConcurrentContainer
from uwds_msgs.msg import Node

ENTITY = Node.ENTITY
MESH = Node.MESH
CAMERA = Node.CAMERA

NodeTypeNames = {ENTITY: "entity", MESH: "mesh", CAMERA: "camera"}

class Nodes(ConcurrentContainer):

    def update(self, nodes):
        super(Nodes, self).update([n.id for n in nodes], nodes)

    def get_node_property(self, node_id, property_name):
        self._lock()
        for property in self[node_id].properties:
            if property.name == property_name:
                self._unlock()
                return property.data
        self._unlock()
        return ""

    def by_property(self, property_name, property_value=None):
        self._lock()
        if property_value is None:
            nodes = [n for n in self if self.get_node_property(n.id, property_name) != ""]
        else:
            nodes = [n for n in self if self.get_node_property(n.id, property_name) == property_value]
        self._unlock()
        return nodes

    def by_name(self, node_name):
        self._lock()
        nodes = [n for n in self if n.name == node_name]
        self._unlock()
        return nodes

    def by_type(self, type):
        self._lock()
        nodes = [n for n in self if n.type == type]
        self._unlock()
        return nodes
