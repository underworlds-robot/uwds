#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from uwds_msgs.srv import QueryInContext
from pyuwds.uwds_client import UwdsClient
from pyuwds.uwds import READER
from pyuwds.types.nodes import CAMERA, MESH, ENTITY
from pyuwds.types.situations import FACT, ACTION, GENERIC, INTERNAL
from pyuwds.tools.glove import GloveManager

class UwdsKBLite(UwdsClient):
    def __init__(self):
        super(UwdsKBLite, self).__init__("uwds_knowledge_base", READER)
        self.__query_service = rospy.Service("uwds/query_knowledge_base", QueryInContext, self.handleQuery)
        data_dir = rospy.get_param("~data_dir", "")
        self.__node_threshold = rospy.get_param("~node_threshold", 0.80)
        self.__situation_threshold = rospy.get_param("~situation_threshold", 0.91)
        self.__action_threshold = rospy.get_param("~action_threshold", 0.85)
        words_to_keep = rospy.get_param("~words_to_keep", "in on under above was below").split(" ")
        dim = rospy.get_param("~dim", 300)
        stoplist = rospy.get_param("~stop_list", 50)
        additional_symbols_path = rospy.get_param("~situation_threshold", 0.85)
        self.__glove = GloveManager(data_dir+"/glove/glove.6B."+str(dim)+"d.txt", stoplist=stoplist, keep=words_to_keep)
        rospy.loginfo("["+self.ctx.name()+"::queryKnowledgeBase] Underworlds KB ready !")

    def onChanges(self, world_name, header, invalidations):
        pass

    def queryKnowledgeBase(self, world_name, query):
        """
        """
        scene = self.ctx.worlds()[world_name].scene()
        timeline = self.ctx.worlds()[world_name].timeline()
        result = []
        for node in scene.nodes():
            str_eval = node.name.replace("_", " ").lower()
            if self.__glove.match(str_eval, query) > self.__node_threshold:
                result.append(node.id)
        if len(result) > 1:
            return result
        else:
            for situation in timeline.situations():
                str_eval = situation.description.replace("_", " ").lower()
                if self.__glove.match(str_eval, query) > self.__situation_threshold:
                    subject = timeline.situations().get_situation_property(situation.id, "subject")
                    if subject != "":
                        result.append(subject)
        return result

    def handleQuery(self, req):
        """
        """
        try:
            result = self.queryKnowledgeBase(req.ctxt.world, req.query)
            return result, True, ""
        except Exception as e:
            rospy.logwarn("["+self.ctx.name()+"::queryKnowledgeBase] Exception occurred : "+str(e))
            return [], False, str(e)

if __name__ == '__main__':
    rospy.init_node("uwds_kb_lite", anonymous=False)
    kb = UwdsKBLite()
    rospy.spin()
