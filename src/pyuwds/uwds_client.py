#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from dynamic_connection_based_node import DynamicConnectionBasedNode
from uwds_msgs.srv import GetTopology, GetScene, GetTimeline, GetMesh
from uwds_msgs.msg import Context, ChangesInContextStamped
from pyuwds import World

DEFAULT_PUBLISHER_BUFFER_SIZE = 10


class UwdsClient(DynamicConnectionBasedNode):
    """
    The Underworlds client

    @type self.node_name: string
    @param self.node_name: The client name
    """
    def __init__(self, node_name, client_type):
        """
        The Underworlds client

        @type self.node_name: string
        @param self.node_name: The client name
        """
        # super
        DynamicConnectionBasedNode.__init__(self)
        # General service
        self.node_name = node_name
        self.client_id = str(uuid.uuid4())
        self.client_type = client_type
        self.ever_send_changes = False

        if rospy.has_param("~use_scene"):
            self.use_scene = rospy.get_param("~use_scene")
        else:
            self.use_scene = True

        if rospy.has_param("~use_timeline"):
            self.use_timeline = rospy.get_param("~use_timeline")
        else:
            self.use_timeline = True

        if rospy.has_param("~use_meshes"):
            self.use_meshes = rospy.get_param("~use_meshes")
        else:
            self.use_meshes = True

        self.get_scene_service_client = rospy.ServiceProxy("uwds/get_topology", GetTopology, persistent=True)
        rospy.logdebug("[%s] Service client 'uwds/get_topology' created", self.node_name)
        # Nodes related service
        self.get_scene_service_client = rospy.ServiceProxy("uwds/get_scene", GetScene, persistent=True)
        rospy.logdebug("[%s] Service client 'uwds/get_scene' created", self.node_name)
        # Situations related service
        self.get_timeline_service_client = rospy.ServiceProxy("uwds/get_timeline", GetTimeline, persistent=True)
        rospy.logdebug("[%s] Service client 'uwds/get_timeline' created", self.node_name)
        # Meshes related service
        self.get_mesh_service_client = rospy.ServiceProxy("uwds/get_mesh", GetMesh, persistent=True)
        rospy.logdebug("[%s] Service client 'uwds/get_mesh' created", self.node_name)
        # Changes publisher
        self.changes_publisher = rospy.Publisher("uwds/changes", ChangesInContextStamped, queue_size=DEFAULT_PUBLISHER_BUFFER_SIZE)
        rospy.logdebug("[%s] Publisher 'uwds/changes' created", self.node_name)

    def sendWorldChanges(self, world_name, header, changes):
        """
        Send changes to Underworlds server

        @typedef world_name: string
        @param world_name: The world name to update
        @typedef header: Header
        @param header: The message header
        @typedef changes: Changes
        @param changes: The changes to send
        """
        msg = ChangesInContextStamped()
        msg.header = header
        msg.ctxt.world = world_name
        msg.ctxt.client.name = self.node_name
        msg.ctxt.client.id = self.client_id
        msg.ctxt.client.type = self.client_type
        msg.changes = changes
        self.changes_publisher.publish(msg)

        if self.ever_send_changes is False:
            self.ever_send_changes = True

    def getTopologyFromRemote():
        """
        """
        ctxt = Context()
        ctxt.client = self.node_name
        rospy.wait_for_service("uwds/get_topology")
        try:
            res = self.get_timeline_service_client(ctxt)
            if(not response.success):
                rospy.logerr("[%s] Error occured while processing 'uwds/get_topology' : %s", self.node_name, res.error)
                return
            # TODO update the topology
        except rospy.ServiceException, e:
            rospy.logerr("[%s] Service 'uwds/get_topology' call failed: %s", self.node_name, e)

    def getSceneFromRemote(self, world_name):
        if(not self.use_scene):
            rospy.logwarn("[%s] Trying to request service 'uwds/get_scene' while '~use_scene' parameter is desactivated. Skip the request.", self.node_name)
        ctxt = Context()
        ctxt.client.id = self.client_id
        ctxt.client.type = self.client_type
        ctxt.client.name = self.node_name
        ctxt.world = world_name
        rospy.wait_for_service("uwds/get_scene")
        try:
            res = self.get_scene_service_client(ctxt)
            if(not response.success):
                rospy.logerr("[%s] Error occured while processing 'uwds/get_scene' : %s", self.node_name, res.error)
                return
            worlds[world_name] = World(world_name, self.meshes)
            worlds[world_name].reset(res.root_id)
            for node in res.nodes:
                if(self.use_meshes):
                    self.getNodeMeshes(node.id)
                worlds[world_name].scene.nodes[node.id] = node
        except rospy.ServiceException, e:
            rospy.logerr("[%s] Service 'uwds/get_scene' call failed: %s", self.node_name, e)


    def getTimelineFromRemote(self, world_name):
        """
        """
        if(not self.use_timeline):
            rospy.logwarn("[%s] Trying to request service 'uwds/get_timeline' while '~use_timeline' parameter is desactivated. Skip the request.", self.node_name)
        ctxt = Context()
        ctxt.client.id = self.client_id
        ctxt.client.type = self.client_type
        ctxt.client.name = self.node_name
        ctxt.world = world_name
        rospy.wait_for_service("uwds/get_timeline")
        try:
            res = self.get_timeline_service_client(ctxt)
            if(not response.success):
                rospy.logerr("[%s] Error occured while processing 'uwds/get_timeline' : %s", self.node_name, res.error)
                return
            if world_name not in worlds:
                self.worlds[world_name] = World(world_name, self.meshes)
            self.worlds[world_name].reset(res.origin)
            for situation in res.situations:
                self.worlds[world_name].timeline.situations[node.id] = situation
        except rospy.ServiceException, e:
            rospy.logerr("[%s] Service 'uwds/get_timeline' call failed: %s", self.node_name, e)

    def initializeWorld(self, world_name):
        """
        @typedef world_name: string
        @param world_name: The world name to update
        """
        self.getSceneFromRemote()
        self.getTimelineFromRemote()

    def getMeshFromRemote(self, mesh_id):
        """
        @typedef mesh_id: string
        @param mesh_id: The ID of the mesh to fetch
        """
        if(not self.use_timeline):
            rospy.logwarn("[%s] Trying to request service 'uwds/get_mesh' while '~use_meshes' parameter is desactivated.", self.node_name)
        rospy.wait_for_service("uwds/get_mesh")
        try:
            res = self.get_mesh_service_client(mesh_id)
            if(not res.success):
                rospy.logerr("[%s] Error occured while processing 'uwds/get_mesh' : %s", self.node_name, res.error)
            else:
                self.meshes[mesh.id] = res.mesh
        except rospy.ServiceException, e:
            rospy.logerr("[%s] Service 'uwds/get_mesh' call failed: %s", self.node_name, e)

    def getNodeMeshes(self, node):
        """
        @typedef node: Node
        @param node: The given node
        """
        mesh_ids = []
        for property in node.properties:
            if property.name == "meshes":
                mesh_ids = property.data.split(",")
                for mesh_id in mesh_ids:
                    if self.getMesh(mesh_id):
                        mesh_ids.append(mesh_id)
        return mesh_ids
