#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from concurrent_container import ConcurrentContainer
from uwds_msgs.msg import Situation
from situations import Situations

class Timeline(object):

    def __init__(self):
        self.reset(rospy.Time.now())

    def update(self, situations):
        current_time = rospy.Time.now()
        for situation in situations:
            situation.last_update.data = current_time
        self.__situations.update(situations)
        return [s.id for s in situations]

    def remove(self, situation_ids):
        removed = []
        removed = self.__situations.remove(situation_ids)
        return situation_ids

    def reset(self, origin):
        self.__origin = origin
        if not hasattr(self, '__situations'):
            situation_ids = []
            self.__situations = Situations()
        else:
            situation_ids = self.__situations.ids()
            self.__situations.reset()
        return situation_ids

    def origin(self):
        return self.__origin

    def situations(self):
        return self.__situations
