#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from uwds_msgs.srv import QueryInContext
from pyuwds.uwds_client import UwdsClient
from pyuwds.uwds import READER
from pyuwds.types.nodes import CAMERA, MESH, ENTITY
from pyuwds.types.situations import FACT, ACTION, GENERIC, INTERNAL
from pyuwds.tools.glove import GloveManager


def getDictValue(elem):
    return elem[1]


class UwdsKBLite(UwdsClient):
    def __init__(self):
        super(UwdsKBLite, self).__init__("uwds_knowledge_base", READER)
        data_dir = rospy.get_param("~data_dir", "")
        self.__match_threshold = rospy.get_param("~match_threshold", 0.85)
        words_to_keep = rospy.get_param("~words_to_keep", "in on under above was below").split(" ")
        dim = rospy.get_param("~dim", 300)
        stoplist = rospy.get_param("~stop_list", 50)
        self.__glove = GloveManager(data_dir+"/glove/glove.6B."+str(dim)+"d.txt", stoplist=stoplist, keep=words_to_keep)
        rospy.loginfo("["+self.ctx.name()+"::queryKnowledgeBase] Underworlds KB ready !")
        self.__query_service = rospy.Service("uwds/query_knowledge_base", QueryInContext, self.handleQuery)

    def onChanges(self, world_name, header, invalidations):
        pass

    def clean_sentence(self, label):
        label = label.replace("_"," ").replace("."," ").replace("-"," ").lower()
        result = []
        for word in label.split(" "):
            try:
                test = int(word)
            except ValueError:
                result.append(word)
        first = True
        for word in result:
            if first is True:
                label = word
                first = False
            else:
                label += " " + word
        return label

    def match(self, sentence1, sentence2):
        clean_sentence1 = self.clean_sentence(sentence1)
        clean_sentence2 = self.clean_sentence(sentence2)
        similarity = self.__glove.match(clean_sentence1, clean_sentence2)
        if(self.verbose):
            print "similarity("+clean_sentence1+" , "+clean_sentence2+") = "+ str(similarity)
        return similarity

    def queryKnowledgeBase(self, world_name, query):
        """
        """
        scene = self.ctx.worlds()[world_name].scene()
        timeline = self.ctx.worlds()[world_name].timeline()

        node_id_to_score = {}

        result = []

        for node in scene.nodes():
            node_id_to_score[node.id] = self.match(node.name, query)
        for situation in timeline.situations():
            if situation.end.data == rospy.Time(0):
                subject = timeline.situations().get_situation_property(situation.id, "subject")
                if subject != "":
                    score = self.match(situation.description, query)
                    if subject in node_id_to_score:
                        if score > node_id_to_score[subject]:
                            node_id_to_score[subject] = score

        match = sorted(node_id_to_score.items(), reverse=True, key=getDictValue)

        for id, score in match:
            if score > self.__match_threshold:
                result.append(id)
            else:
                return result
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
