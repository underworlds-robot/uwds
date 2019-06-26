#!/usr/bin/env python
# -*- coding: utf-8 -*-
from concurrent_container import ConcurrentContainer
from uwds_msgs.msg import Situation

GENERIC = Situation.GENERIC
FACT = Situation.FACT
ACTION = Situation.ACTION
INTERNAL = Situation.INTERNAL

SituationTypeNames = {GENERIC: "generic", FACT: "fact", ACTION: "action", INTERNAL: "internal"}

class Situations(ConcurrentContainer):

    def update(self, situations):
        super(Situations, self).update([s.id for s in situations], situations)

    def get_situation_property(self, situation_id, property_name):
        self._lock()
        if situation_id in self:
            for property in self[situation_id].properties:
                if property.name == property_name:
                    self._unlock()
                    return property.data
        self._unlock()
        return ""

    def by_property(self, property_name, property_value=None):
        self._lock()
        if property_value is None:
            situations = [s for s in self if self.get_situation_property(s.id, property_name) != ""]
        else:
            situations = [s for s in self if self.get_situation_property(s.id, property_name) == property_value]
        self._unlock()
        return situations

    def by_name(self, node_name):
        self._lock()
        situations = [s for s in self if s.name == node_name]
        self._unlock()
        return situations

    def by_type(self, type):
        self._lock()
        situations = [s for s in self if s.type == type]
        self._unlock()
        return situations
