#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
from enum import Enum
import rospy
import uwds_msgs
from uwds_msgs.srv import GetTopology
import pygraphviz as pgv

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Simple introspection tool to visualize the clients topology")
    args = parser.parse_args()
    rospy.init_node("uwds_view_topology", anonymous=False)
    G = pgv.AGraph(strict=True, directed=True)
    # nodes attributes
    G.node_attr['style']="filled"
    G.node_attr["fontname"] = "helvetica"
    # edges attributes
    G.edge_attr['color'] = "black"
    # graph attributes
    G.graph_attr["label"] = "The Underworlds clients topology"
    rospy.wait_for_service("uwds/get_topology")
    get_topology_service_client = rospy.ServiceProxy("uwds/get_topology", GetTopology, persistent=True)
    response = get_topology_service_client()
    client_map = {}
    if response.worlds:
        for world in response.worlds:
            if world:
                G.add_node(world, shape="rectangle", color="orange", fontcolor="white")
    if response.worlds:
        for client in response.clients:
            G.add_node(client.name, fontcolor="black")
            client_map[client.id] = client
    for client_interaction in response.client_interactions:
        if client_interaction.ctxt.world and client_interaction.ctxt.client.id:
            if client_interaction.type == 0:
                edge_from = client_interaction.ctxt.world
                edge_to = client_map[client_interaction.ctxt.client.id].name
            else:
                edge_from = client_map[client_interaction.ctxt.client.id].name
                edge_to = client_interaction.ctxt.world
            G.add_edge(edge_from, edge_to)

    G.layout(prog='dot')
    G.draw("topology.pdf")
