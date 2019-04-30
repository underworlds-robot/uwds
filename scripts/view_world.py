#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
from enum import Enum
import rospy
import uwds_msgs
from uwds_msgs.srv import GetScene, GetTimeline
import pygraphviz as pgv

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Simple introspection tool to visualize the world graph of the given world")
    parser.add_argument("world", help="The Underworlds world to monitor")
    args = parser.parse_args()

    rospy.init_node("uwds_world_graph", anonymous=False)
    G = pgv.AGraph(strict=True, directed=True)
    # nodes attributes
    G.node_attr['style']= "filled"
    G.node_attr["fontname"] = "helvetica"
    # edges attributes
    G.edge_attr['color'] = "black"
    # graph attributes
    G.graph_attr["label"] = "The graph of the world <%s>" % args.world
    rospy.wait_for_service("uwds/get_scene")
    rospy.wait_for_service("uwds/get_timeline")
    get_scene_client = rospy.ServiceProxy("uwds/get_scene", GetScene, persistent=False)
    get_timeline_client = rospy.ServiceProxy("uwds/get_timeline", GetTimeline, persistent=False)
    request_scene = uwds_msgs.srv.GetSceneRequest()
    request_timeline = uwds_msgs.srv.GetTimelineRequest()
    request_scene.ctxt.world = args.world
    request_timeline.ctxt.world = args.world
    scene_response = get_scene_client(request_scene)
    timeline_response = get_timeline_client(request_timeline)
    parent_map = {}
    node_map = {}
    for node in scene_response.nodes:
        G.add_node(node.id, fontcolor="black", label=node.name)
        if node.parent != "":
            parent_map[node.id] = node.parent
            node_map[node.id] = node
    for node_id, parent_id in parent_map.items():
        edge_from = parent_id
        edge_to = node_id
        G.add_edge(edge_from, edge_to, color="gray")

    for situation in timeline_response.situations:
        if situation.end.data == rospy.Time(0):
            edge_from = ""
            edge_to = ""
            label = ""
            for property in situation.properties:
                if property.name == "subject":
                    edge_from = property.data
                if property.name == "object":
                    edge_to = property.data
                if property.name == "predicate" or property.name == "action":
                    label = property.data
            if edge_from != "" and edge_to != "":
                G.add_edge(edge_from, edge_to, label=label)
    G.layout(prog='dot')
    G.draw("world.pdf")
