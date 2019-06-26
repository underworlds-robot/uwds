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

    def remove(self, node_ids):
        removed = []
        for node_id in node_ids:
            if node_id in self:
                parent_node = self[self[node_id].parent]
                to_remove = []
                for i in range(0, len(parent_node.children)):
                    if parent_node.children[i] == node_id:
                        to_remove.append(i)
                for id in to_remove:
                    del parent_node.children[id]
                self.update([parent_node])
                self.delete(node_id)
                removed.append(node_id)
        return removed

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
