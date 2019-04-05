#!/usr/bin/env python
# -*- coding: utf-8 -*-

from scene import Scene
from timeline import Timeline
from uwds_msgs.msg import Invalidations

class World:
    """
    The Underworlds world data structure
    """
    def __init__(self, name, meshes):
        self.__scene = Scene()
        self.__timeline = Timeline()
        self.__meshes = meshes
        self.__name = name

    def apply_changes(self, header, changes):
        invalidations = Invalidations()

        invalidations.node_ids_deleted = self.__scene.remove(changes.nodes_to_delete)
        invalidations.node_ids_updated = self.__scene.update(changes.nodes_to_update)

        invalidations.situation_ids_deleted = self.__timeline.remove(changes.situations_to_delete)
        invalidations.situation_ids_updated = self.__timeline.update(changes.situations_to_update)

        for mesh_id in changes.meshes_to_delete:
            del self.__meshes[mesh_id]
            invalidations.mesh_ids_deleted.append(mesh_id)
        for mesh in changes.meshes_to_update:
            self.__meshes[mesh.id] = mesh
            invalidations.mesh_ids_updated.append(mesh.id)
        return invalidations

    def reset(self):
        self.__scene.reset(self.__scene.root_id())
        self.__timeline.reset(self.__timeline.origin().data)
        self.__meshes.reset()
