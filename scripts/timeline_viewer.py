#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from pyuwds.reconfigurable_client import ReconfigurableClient
from pyuwds.uwds import READER
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA


class TimelineViewer(ReconfigurableClient):
    """
    """
    def __init__(self):
        """
        """
        self.__text_pub = {}
        self.__overlay_name = rospy.get_param("~overlay_name", "timeline viewer")
        super(TimelineViewer, self).__init__("timeline_viewer", READER)

        rospy.Timer(rospy.Duration(1/30.0), self.handleTimer)

    def onReconfigure(self, worlds_names):
        """
        """
        #rospy.loginfo("reconfigure")
        text = OverlayText()

        text.action = 1
        for world_name in worlds_names:
            scene = self.ctx.worlds()[world_name].scene()
            timeline = self.ctx.worlds()[world_name].timeline()
            if world_name not in self.__text_pub:
                self.__text_pub[world_name] = rospy.Publisher(world_name+"/timeline", OverlayText, queue_size=2)
            self.__text_pub[world_name].publish(text)
        pass

    def onSubscribeChanges(self, world_name):
        """
        """
        pass

    def onUnsubscribeChanges(self, world_name):
        """
        """
        pass

    def onChanges(self, world_name, header, invalidations):
        """
        """
        pass

    def handleTimer(self, event):
        self.publishOverlaytext(self.input_worlds[0])

    def publishOverlaytext(self, world_name):
        """
        """
        situations_text = " <"+world_name+"> "+self.__overlay_name +"\n\r"
        situations_text += "- facts\n\r"
        situations_text += "id : description\n\r"
        situations_text += "----------------\n\r"
        fact_sorted = {}
        fact_desc = {}
        for situation in self.ctx.worlds()[world_name].timeline().situations():
            if situation.end.data != situation.start.data:
                if situation.end.data == rospy.Time(0):
                    if situation.description not in fact_desc:
                        fact_sorted[situation.start.data] = situation
                        fact_desc[situation.description] = situation
                else:
                    if rospy.Time.now() - situation.end.data < rospy.Duration(5.0):
                        if situation.description not in fact_desc:
                            fact_sorted[situation.start.data] = situation
                            fact_desc[situation.description] = situation

        for key in sorted(fact_sorted.iterkeys()):
            situations_text += fact_sorted[key].id[:5]+" : "+fact_sorted[key].description +"\n\r"

        situations_text += "\n\r- events\n\r"
        situations_text += "id : description\n\r"
        situations_text += "----------------\n\r"
        event_sorted = {}
        for situation in self.ctx.worlds()[world_name].timeline().situations():
            if situation.start.data == situation.end.data:
                if rospy.Time.now() - situation.end.data < rospy.Duration(5.0):
                    event_sorted[situation.start.data] = situation

        for key in sorted(event_sorted.iterkeys()):
                situations_text += event_sorted[key].id[:5]+" : "+event_sorted[key].description +"\n\r"

        text = OverlayText()
        text.width = 400
        text.height = 600
        text.left = 10
        text.top = 10
        text.text_size = 12
        text.line_width = 2
        text.font = "DejaVu Sans Mono"
        text.text = situations_text
        text.fg_color = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
        text.bg_color = ColorRGBA(0.0, 0.0, 0.0, 0.2)

        if world_name not in self.__text_pub:
            self.__text_pub[world_name] = rospy.Publisher(world_name+"/timeline", OverlayText, queue_size=2)
        self.__text_pub[world_name].publish(text)

if __name__ == '__main__':
    rospy.init_node("timeline_viewer")
    viewer = TimelineViewer()
    rospy.spin()
