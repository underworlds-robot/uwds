#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from concurrent_container import ConcurrentContainer
from uwds_msgs.msg import Situation

GENERIC = Situation.GENERIC
FACT = Situation.FACT
ACTION = Situation.ACTION
INTERNAL = Situation.INTERNAL

SituationTypeNames = {GENERIC: "generic",
                 FACT: "fact",
                 ACTION: "action",
                 INTERNAL: "internal"}

class Timeline(object):

    def __init__(self):
        self.reset(rospy.Time.now())

    def update(self, situations):
        current_time = rospy.Time.now()
        for situation in situations:
            situation.last_update.data = current_time
        self.__situations.update([s.id for s in situations], situations)
        return [s.id for s in situations]

    def remove(self, situation_ids):
        self.__situations.remove(situation_ids)

    def reset(self, origin):
        self.__origin = origin
        if not hasattr(self, '__situations'):
            situation_ids = []
            self.__situations = ConcurrentContainer()
        else:
            situation_ids = self.__situations.ids()
            self.__situations.reset()
        return situation_ids

    def origin(self):
        return self.__origin

    '''
    def getSituationProperty(self, situation_id, property_name):
        if situation_id in self.situations:
            for property in self.situations[situation_id].properties:
                if property.name == property_name:
                    return property.data
        return ""

    def get_situation_property(self, situation_id, property_name):
        self._lock()
        if self.__situations.has(situation_id):
            for property in self.__situations[situation_id].properties:
                if property.name == property_name:
                    self._unlock()
                    return property.data
        self._unlock()
        return ""
    '''
