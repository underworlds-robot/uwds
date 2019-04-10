#!/usr/bin/env python
# -*- coding: utf-8 -*-
from types.gen_uuid import gen_uuid
from uwds_msgs.msg import Client

from proxy.meshes_proxy import MeshesProxy
from proxy.topology_proxy import TopologyProxy
from proxy.worlds_proxy import WorldsProxy

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


class UnderworldsProxy(object):

    def __init__(self, client_name, client_type):
        self.__client = Client(name=client_name, id=gen_uuid(), type=client_type)
        self.__meshes_proxy = MeshesProxy(self.__client)
        self.__worlds_proxy = WorldsProxy(self.__client, self.__meshes_proxy)
        self.__topology_proxy = TopologyProxy(self.__client)

    def worlds(self):
        return self.__worlds_proxy

    def topology(self):
        self.__topology_proxy.get_topology_from_remote()
        return self.__topology_proxy.topology()

    def meshes(self):
        return self.__meshes_proxy.meshes()

    def name(self):
        return self.__client.name
