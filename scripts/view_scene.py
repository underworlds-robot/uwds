#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
from enum import Enum
import rospy
import uwds_msgs
from uwds_msgs.srv import GetScene
import pygraphviz as pgv

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Simple introspection tool to visualize the scene of the given world")
    parser.add_argument("world", help="The Underworlds world to monitor")
    args = parser.parse_args()

    rospy.init_node("uwds_view_scene_graph", anonymous=False)
    G = pgv.AGraph(strict=True, directed=True)
    # nodes attributes
    G.node_attr['style']= "filled"
    G.node_attr["fontname"] = "helvetica"
    # edges attributes
    G.edge_attr['color'] = "black"
    # graph attributes
    G.graph_attr["label"] = "The scene of the world <%s>" % args.world
    rospy.wait_for_service("uwds/get_scene")
    get_scene_client = rospy.ServiceProxy("uwds/get_scene", GetScene, persistent=True)
    request = uwds_msgs.srv.GetSceneRequest()
    request.ctxt.world = args.world
    response = get_scene_client(request)

    parent_map = {}
    node_names_map = {}
    nodes_map = {}
    for node in response.nodes:
        G.add_node(node.name, fontcolor="black")
        node_names_map[node.id] = node.name
        nodes_map[node.id] = node
        if node.parent != "":
            parent_map[node.id] = node.parent
    for node_id, parent_id in parent_map.items():
        edge_from = node_names_map[parent_id]
        edge_to = node_names_map[node_id]
        duration = rospy.Time.now() - node.last_observation.data
        time_str = ""
        G.add_edge(edge_from, edge_to)
    G.layout(prog='dot')
    G.draw("scene.pdf")
