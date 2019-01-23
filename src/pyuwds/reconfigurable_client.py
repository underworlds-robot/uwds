#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import message_filters
from uwds_client import UwdsClient
from dynamic_connection_based_node import CONNECTED, NOT_CONNECTED, CONNECTING
from uwds_msgs.srv import ReconfigureInputs
from uwds_msgs.msg import ChangesInContextStamped, Invalidations
from std_msgs.msg import Header


class ReconfigurableClient(UwdsClient):
    """
    This class allow to dynamically reconfigure input worlds topics.
    """
    def __init__(self, node_name, client_type):
        """
        The constructor to call by subclass.

        @typedef node_name: string
        @param node_name: The node name
        """
        UwdsClient.__init__(self, node_name, client_type)

        self.synchronized = rospy.get_param("~synchronized", False)
        self.ever_connected = False

        if rospy.has_param("~default_inputs"):
            default_inputs = rospy.get_param("~default_inputs")
            rospy.loginfo("[%s::init] Connecting the node to default inputs : %s", self.node_name, default_inputs)
            default_inputs_list = default_inputs.split(" ")
            try:
                self.reconfigure(default_inputs_list)
                self.connection_status = NOT_CONNECTED
            except Exception as e:
                rospy.logerr("[%s::reconfigure] Error occured : %s", self.node_name, str(e))
                self.connection_status = NOT_CONNECTED
        else:
            self.connection_status = NOT_CONNECTED

        self.reconfigure_service_server = rospy.Service("~reconfigure_inputs",
                                                        ReconfigureInputs,
                                                        self.reconfigureInputs)
        if(self.verbose):
            rospy.loginfo("[%s::init] Service '~reconfigure_inputs' advertised",
                          self.node_name)
        self.active_sync_connection = None
        self.time_synchronizer = None

    def reconfigure(self, new_input_world_names):
        """
        """
        header = Header()
        header.frame_id = self.global_frame_id
        header.stamp = rospy.Time.now()
        rospy.loginfo("[%s::reconfigure] Reconfiguring", self.node_name)
        self.connection_mutex.acquire()
        if len(new_input_world_names) == 0:
            if self.connection_status == CONNECTED:
                rospy.logwarn("[%s::reconfigure] Disconnecting the node", self.node_name)
                if(self.synchronized):
                    self.active_sync_connection = None
                for input_world in self.input_worlds:
                    self.removeChangesPublisher(input_world)
                    self.onUnsubscribeChanges(input_world)
                self.resetInputWorlds()
                self.resetOutputWorlds()
                self.connection_status = NOT_CONNECTED
            self.connection_mutex.release()
            return(True)
        self.connection_status = CONNECTING
        if self.synchronized is True:
            if(len(new_input_world_names) > 8 or len(new_input_world_names) < 2):
                self.connection_mutex.release()
                raise ValueError("Incorrect number of inputs specified")
                return(False)
        rospy.loginfo("[%s::reconfigure] Remove old connections if any", self.node_name)
        for input_world in self.input_worlds:
            self.removeChangesSubscriber(input_world)
            self.onUnsubscribeChanges(input_world)
        self.resetInputWorlds()
        self.resetOutputWorlds()
        rospy.loginfo("[%s::reconfigure] Add new connections", self.node_name)
        self.onReconfigure(new_input_world_names)
        for new_input_world in new_input_world_names:
            rospy.loginfo("[%s::reconfigure] Initialize world <%s>", self.node_name, new_input_world)
            invalidations = self.initializeWorld(new_input_world)
            self.addChangesSubscriber(new_input_world)
            self.onSubscribeChanges(new_input_world)
            rospy.loginfo("[%s::reconfigure] make changes for world <%s>", self.node_name, new_input_world)
            self.onChanges(new_input_world, header, invalidations)
        #self.onReconfigure(new_input_world_names)
        if self.synchronized:
            if len(new_input_world_names) == 2:
                self.time_synchronizer = message_filters.TimeSynchronizer(
                    [self.sync_changes_subscribers_map[new_input_world_names[0]],
                     self.sync_changes_subscribers_map[new_input_world_names[1]]], self.time_synchronizer_buffer_size)
            if len(new_input_world_names) == 3:
                self.time_synchronizer = message_filters.TimeSynchronizer(
                    [self.sync_changes_subscribers_map[new_input_world_names[0]],
                     self.sync_changes_subscribers_map[new_input_world_names[1]],
                     self.sync_changes_subscribers_map[new_input_world_names[2]]], self.time_synchronizer_buffer_size)
            if len(new_input_world_names) == 4:
                self.time_synchronizer = message_filters.TimeSynchronizer(
                    [self.sync_changes_subscribers_map[new_input_world_names[0]],
                     self.sync_changes_subscribers_map[new_input_world_names[1]],
                     self.sync_changes_subscribers_map[new_input_world_names[2]],
                     self.sync_changes_subscribers_map[new_input_world_names[3]]], self.time_synchronizer_buffer_size)
            if len(new_input_world_names) == 5:
                self.time_synchronizer = message_filters.TimeSynchronizer(
                    [self.sync_changes_subscribers_map[new_input_world_names[0]],
                     self.sync_changes_subscribers_map[new_input_world_names[1]],
                     self.sync_changes_subscribers_map[new_input_world_names[2]],
                     self.sync_changes_subscribers_map[new_input_world_names[3]],
                     self.sync_changes_subscribers_map[new_input_world_names[4]]], self.time_synchronizer_buffer_size)
            if len(new_input_world_names) == 6:
                self.time_synchronizer = message_filters.TimeSynchronizer(
                    [self.sync_changes_subscribers_map[new_input_world_names[0]],
                     self.sync_changes_subscribers_map[new_input_world_names[1]],
                     self.sync_changes_subscribers_map[new_input_world_names[2]],
                     self.sync_changes_subscribers_map[new_input_world_names[3]],
                     self.sync_changes_subscribers_map[new_input_world_names[4]],
                     self.sync_changes_subscribers_map[new_input_world_names[5]]], self.time_synchronizer_buffer_size)
            if len(new_input_world_names) == 7:
                self.time_synchronizer = message_filters.TimeSynchronizer(
                    [self.sync_changes_subscribers_map[new_input_world_names[0]],
                     self.sync_changes_subscribers_map[new_input_world_names[1]],
                     self.sync_changes_subscribers_map[new_input_world_names[2]],
                     self.sync_changes_subscribers_map[new_input_world_names[3]],
                     self.sync_changes_subscribers_map[new_input_world_names[4]],
                     self.sync_changes_subscribers_map[new_input_world_names[5]],
                     self.sync_changes_subscribers_map[new_input_world_names[6]]], self.time_synchronizer_buffer_size)
            if len(new_input_world_names) == 8:
                self.time_synchronizer = message_filters.TimeSynchronizer(
                 [self.sync_changes_subscribers_map[new_input_world_names[0]],
                  self.sync_changes_subscribers_map[new_input_world_names[1]],
                  self.sync_changes_subscribers_map[new_input_world_names[2]],
                  self.sync_changes_subscribers_map[new_input_world_names[3]],
                  self.sync_changes_subscribers_map[new_input_world_names[4]],
                  self.sync_changes_subscribers_map[new_input_world_names[5]],
                  self.sync_changes_subscribers_map[new_input_world_names[6]],
                  self.sync_changes_subscribers_map[new_input_world_names[7]]], self.time_synchronizer_buffer_size)
        self.connection_status = CONNECTED
        if self.ever_connected is False:
            self.ever_connected = True
        self.connection_mutex.release()
        return(True)

    def reconfigureInputs(self, req):
        """
        This method is called when a reconfigure service is received.

        @typedef req: ListRequest
        @param req: The reconfigure request
        """
        if self.verbose:
            rospy.loginfo(
                "[%s::reconfigureInputs] Service ~reconfigure_inputs requested",
                self.node_name)
        try:
            self.reconfigure(req.inputs)
        except Exception as e:
            rospy.logerr("[%s::reconfigureInputs] Error occured : %s", self.node_name, str(e))
            return(False, e)
        return(True, "")

    def applyChanges(self, changes_list):
        """
        This method is called when a changes in a world are received.

        @typedef changes_list: ChangesInContextStampedConstPtr
        @param changes_list: The message list received
        """
        #rospy.loginfo("[%s::applyChanges] Received changes from server", self.node_name)
        for changes_in_ctxt in changes_list:
            invalidations = self.worlds[changes_in_ctxt.ctxt.world].applyChanges(changes_in_ctxt.header, changes_in_ctxt.changes)
            self.onChanges(changes_in_ctxt.ctxt.world, changes_in_ctxt.header, invalidations)

    def onReconfigure(self, world_names):
        """
        This method is called at the end of the reconfigure process.
        Add additional initialization in this method.

        @typedef world_names: string list
        @param world_names: The new input world names
        """
        raise NotImplementedError

    def onSubscribeChanges(self, world_name):
        """
        This method is called when a world changes is subscribed by this nodelet.
        Set up additional subscribers or class in this method.

        @typedef world_name: string
        @param world_name: The name of the world subscribed
        """
        raise NotImplementedError

    def onUnsubscribeChanges(self, world_name):
        """
        This method is called when a world changes is unsubscribed by this nodelet.
        Shut down additional subscribers or class in this method.

        @typedef world: string
        @param world The name of the world unsubscribed
        """
        raise NotImplementedError

    def onChanges(self, world_name, header, invalidations):
        """
        This method is called when there is a change in a world,
        subclass need to implement this method.
        """
        raise NotImplementedError

    def changesCallback0(self, msg):
        """
        This method is called when a changes in a world are received.

        @typedef msg0: ChangesInContextStampedConstPtr
        @param msg0: The 1st message received
        @typedef msg1: ChangesInContextStampedConstPtr
        @param msg1: The 2nd message received
        """
        changes_list = []
        changes_list.append(msg)
        self.applyChanges(changes_list)

    def changesCallback1(self, msg0, msg1):
        """
        This method is called when a changes in a world are received.

        @typedef msg0: ChangesInContextStampedConstPtr
        @param msg0: The 1st message received
        @typedef msg1: ChangesInContextStampedConstPtr
        @param msg1: The 2nd message received
        @typedef msg2: ChangesInContextStampedConstPtr
        @param msg2: The 3nd message received
        """
        changes_list = []
        changes_list.append(msg0)
        changes_list.append(msg1)
        self.applyChanges(changes_list)

    def changesCallback2(self, msg0, msg1, msg2):
        """
        This method is called when a changes in a world are received.

        @typedef msg0: ChangesInContextStampedConstPtr
        @param msg0: The 1st message received
        @typedef msg1: ChangesInContextStampedConstPtr
        @param msg1: The 2nd message received
        @typedef msg2: ChangesInContextStampedConstPtr
        @param msg2: The 3nd message received
        @typedef msg3: ChangesInContextStampedConstPtr
        @param msg3: The 3nd message received
        """
        changes_list = []
        changes_list.append(msg0)
        changes_list.append(msg1)
        changes_list.append(msg2)
        self.applyChanges(changes_list)

    def changesCallback3(self, msg0, msg1, msg2, msg3):
        """
        This method is called when a changes in a world are received.

        @typedef msg0: ChangesInContextStampedConstPtr
        @param msg0: The 1st message received
        @typedef msg1: ChangesInContextStampedConstPtr
        @param msg1: The 2nd message received
        @typedef msg2: ChangesInContextStampedConstPtr
        @param msg2: The 3nd message received
        @typedef msg3: ChangesInContextStampedConstPtr
        @param msg3: The 4nd message received
        @typedef msg4: ChangesInContextStampedConstPtr
        @param msg4: The 5nd message received
        """
        changes_list = []
        changes_list.append(msg0)
        changes_list.append(msg1)
        changes_list.append(msg2)
        changes_list.append(msg3)
        self.applyChanges(changes_list)
        raise NotImplementedError

    def changesCallback4(self, msg0, msg1, msg2, msg3, msg4):
        """
        This method is called when a changes in a world are received.

        @typedef msg0: ChangesInContextStampedConstPtr
        @param msg0: The 1st message received
        @typedef msg1: ChangesInContextStampedConstPtr
        @param msg1: The 2nd message received
        @typedef msg2: ChangesInContextStampedConstPtr
        @param msg2: The 3nd message received
        @typedef msg3: ChangesInContextStampedConstPtr
        @param msg3: The 4nd message received
        @typedef msg4: ChangesInContextStampedConstPtr
        @param msg4: The 5nd message received
        """
        changes_list = []
        changes_list.append(msg0)
        changes_list.append(msg1)
        changes_list.append(msg2)
        changes_list.append(msg3)
        changes_list.append(msg4)
        self.applyChanges(changes_list)

    def changesCallback5(self, msg0, msg1, msg2, msg3, msg4):
        """
        This method is called when a changes in a world are received.

        @typedef msg0: ChangesInContextStampedConstPtr
        @param msg0: The 1st message received
        @typedef msg1: ChangesInContextStampedConstPtr
        @param msg1: The 2nd message received
        @typedef msg2: ChangesInContextStampedConstPtr
        @param msg2: The 3nd message received
        @typedef msg3: ChangesInContextStampedConstPtr
        @param msg3: The 4nd message received
        @typedef msg4: ChangesInContextStampedConstPtr
        @param msg4: The 5nd message received
        @typedef msg5: ChangesInContextStampedConstPtr
        @param msg5: The 6nd message received
        """
        changes_list = []
        changes_list.append(msg0)
        changes_list.append(msg1)
        changes_list.append(msg2)
        changes_list.append(msg3)
        changes_list.append(msg4)
        self.applyChanges(changes_list)

    def changesCallback6(self, msg0, msg1, msg2, msg3, msg4, msg5):
        """
        This method is called when a changes in a world are received.

        @typedef msg0: ChangesInContextStampedConstPtr
        @param msg0: The 1st message received
        @typedef msg1: ChangesInContextStampedConstPtr
        @param msg1: The 2nd message received
        @typedef msg2: ChangesInContextStampedConstPtr
        @param msg2: The 3nd message received
        @typedef msg3: ChangesInContextStampedConstPtr
        @param msg3: The 4nd message received
        @typedef msg4: ChangesInContextStampedConstPtr
        @param msg4: The 5nd message received
        @typedef msg5: ChangesInContextStampedConstPtr
        @param msg5: The 6nd message received
        @typedef msg6: ChangesInContextStampedConstPtr
        @param msg6: The 7nd message received
        """
        changes_list = []
        changes_list.append(msg0)
        changes_list.append(msg1)
        changes_list.append(msg2)
        changes_list.append(msg3)
        changes_list.append(msg4)
        changes_list.append(msg5)
        self.applyChanges(changes_list)

    def changesCallback7(self, msg0, msg1, msg2, msg3, msg4, msg5, msg6):
        """
        This method is called when a changes in a world are received.

        @typedef msg0: ChangesInContextStampedConstPtr
        @param msg0: The 1st message received
        @typedef msg1: ChangesInContextStampedConstPtr
        @param msg1: The 2nd message received
        @typedef msg2: ChangesInContextStampedConstPtr
        @param msg2: The 3nd message received
        @typedef msg3: ChangesInContextStampedConstPtr
        @param msg3: The 4nd message received
        @typedef msg4: ChangesInContextStampedConstPtr
        @param msg4: The 5nd message received
        @typedef msg5: ChangesInContextStampedConstPtr
        @param msg5: The 6nd message received
        @typedef msg6: ChangesInContextStampedConstPtr
        @param msg6: The 7nd message received
        """
        changes_list = []
        changes_list.append(msg0)
        changes_list.append(msg1)
        changes_list.append(msg2)
        changes_list.append(msg3)
        changes_list.append(msg4)
        changes_list.append(msg5)
        changes_list.append(msg6)
        self.applyChanges(changes_list)

    def changesCallback8(self, msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7):
        """
        This method is called when a changes in a world are received.

        @typedef msg0: ChangesInContextStampedConstPtr
        @param msg0: The 1st message received
        @typedef msg1: ChangesInContextStampedConstPtr
        @param msg1: The 2nd message received
        @typedef msg2: ChangesInContextStampedConstPtr
        @param msg2: The 3nd message received
        @typedef msg3: ChangesInContextStampedConstPtr
        @param msg3: The 4nd message received
        @typedef msg4: ChangesInContextStampedConstPtr
        @param msg4: The 5nd message received
        @typedef msg5: ChangesInContextStampedConstPtr
        @param msg5: The 6nd message received
        @typedef msg6: ChangesInContextStampedConstPtr
        @param msg6: The 7nd message received
        @typedef msg7: ChangesInContextStampedConstPtr
        @param msg7: The 8nd message received
        """
        changes_list = []
        changes_list.append(msg0)
        changes_list.append(msg1)
        changes_list.append(msg2)
        changes_list.append(msg3)
        changes_list.append(msg4)
        changes_list.append(msg5)
        changes_list.append(msg6)
        changes_list.append(msg7)
        self.applyChanges(changes_list)

    def addChangesSubscriber(self, world_name):
        """
        This method is called to subscribe to the given world changes.

        @typedef world: string
        @param world: The world to subscribe
        """
        self.addInputWorld(world_name)
        added = False
        if self.synchronized:
            if world_name not in self.sync_changes_subscribers_map:
                self.sync_changes_subscribers_map[world_name] = \
                    message_filters.Subscriber(world_name+"/changes", ChangesInContextStamped)
                added = True
        else:
            if world_name not in self.changes_subscriber_map:
                self.changes_subscriber_map[world_name] = \
                    rospy.Subscriber(world_name+"/changes", ChangesInContextStamped, self.changesCallback0)
                added = True
        if added and self.verbose:
            rospy.loginfo("[%s::addChangesSubscriber] Add changes subscriber for world <%s>", self.node_name, world_name)

    def removeChangesSubscriber(self, world_name):
        """
        This method is called to remove the given world changes subscriber.

        @typedef world: string
        @param world: The world to unsubscribe
        """
        removed = False
        if self.synchronized:
            if world_name in self.sync_changes_subscribers_map:
                del self.sync_changes_subscribers_map[world_name]
                removed = True
        else:
            if world_name not in self.changes_subscriber_map:
                del self.changes_subscriber_map[world_name]
                removed = True
        if removed and self.verbose:
            rospy.loginfo("[%s::removeChangesSubscriber] Remove changes subscriber for world <%s>", self.node_name, world_name)
