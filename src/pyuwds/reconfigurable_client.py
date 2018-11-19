#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from uwds_msgs.srv import ReconfigureInputs


class ReconfigurableClient(UwdsClient):
    """
    This class allow to dynamically reconfigure input worlds topics.
    """
    def __init__(self, node_name):
        """
        The constructor to call by subclass.

        @typedef node_name: string
        @param node_name: The node name
        """
        UwdsClient.__init__(self, node_name)

        if rospy.has_param("~synchronized"):
            self.syncronized = rospy.get_param("~syncronized")
        else:
            self.syncronized = False
        self.ever_connected = False

        if rospy.has_param("~default_inputs"):
            default_inputs = rospy.get_param("~default_inputs")
            default_inputs_list = default_inputs.split(" ")
            try:
                reconfigure(default_inputs_list)
                self.connection_status = ConnectionStatus.NOT_CONNECTED
            except Exception e:
                rospy.logerr("[%s] Error occured : %s" self.node_name, e)
                self.connection_status = ConnectionStatus.NOT_CONNECTED
        else:
            self.connection_status = ConnectionStatus.NOT_CONNECTED

        self.reconfigure_service_server = rospy.Service("~reconfigure_inputs", ReconfigureInputs, self.reconfigureInputs)
        if(self.verbose):
            rospy.loginfo("[%s] Service '~reconfigure_inputs' advertised", self.node_name)
        self.active_sync_connection = None
        self.time_synchronizer = None

    def reconfigure(new_input_worlds):
        """
        """
        self.connection_mutex.aquire()
        if req.inputs.length() == 0:
            if self.connection_status == ConnectionStatus.CONNECTED:
                rospy.logwarn("[%s] Disconnecting the node", self.node_name)
                if(self.syncronized):
                    self.active_sync_connection = None
                for input_world in self.input_worlds:
                    self.removeChangesPublisher(input_world)
                    self.onUnsubscribeChanges(input_world)
                self.resetInputWorlds()
                self.resetOutputWorlds()
                self.connection_status = ConnectionStatus.NOT_CONNECTED
            self.connection_mutex.release()
            return(True)
        connection_status = ConnectionStatus.CONNECTING
        if self.synchronized:
            if(req.inputs.length() > 8 or req.inputs.length() < 2):
                connection_mutex.release()
                raise ValueError("Incorrect number of inputs specified")
                return(False)
        for input_world in self.input_worlds:
            self.removeChangesSubscriber(input_world)
            self.onUnsubscribeChanges(input_world)
        self.resetInputWorlds()
        self.resetOutputWorlds()
        for new_input_world in new_input_worlds:
            initializeWorld(new_input_world)
            addChangesSubscriber(new_input_world)
            onSubscribeChanges(new_input_world)
        if self.syncronized:
            if req.inputs.length() == 2:
                self.time_synchronizer = message_filters.TimeSynchronizer(
                    [self.sync_changes_subscribers_map[res.inputs[0]],
                     self.sync_changes_subscribers_map[res.inputs[1]]], self.time_synchronizer_buffer_size)
            if req.inputs.length() == 3:
                self.time_synchronizer = message_filters.TimeSynchronizer(
                    [self.sync_changes_subscribers_map[res.inputs[0]],
                     self.sync_changes_subscribers_map[res.inputs[1]],
                     self.sync_changes_subscribers_map[res.inputs[2]]], self.time_synchronizer_buffer_size)

            if req.inputs.length() == 4:
                self.time_synchronizer = message_filters.TimeSynchronizer(
                    [self.sync_changes_subscribers_map[res.inputs[0]],
                     self.sync_changes_subscribers_map[res.inputs[1]],
                     self.sync_changes_subscribers_map[res.inputs[2]],
                     self.sync_changes_subscribers_map[res.inputs[3]]], self.time_synchronizer_buffer_size)
            if req.inputs.length() == 5:
                self.time_synchronizer = message_filters.TimeSynchronizer(
                    [self.sync_changes_subscribers_map[res.inputs[0]],
                     self.sync_changes_subscribers_map[res.inputs[1]],
                     self.sync_changes_subscribers_map[res.inputs[2]],
                     self.sync_changes_subscribers_map[res.inputs[3]],
                     self.sync_changes_subscribers_map[res.inputs[4]]], self.time_synchronizer_buffer_size)
            if req.inputs.length() == 6:
                self.time_synchronizer = message_filters.TimeSynchronizer(
                    [self.sync_changes_subscribers_map[res.inputs[0]],
                     self.sync_changes_subscribers_map[res.inputs[1]],
                     self.sync_changes_subscribers_map[res.inputs[2]],
                     self.sync_changes_subscribers_map[res.inputs[3]],
                     self.sync_changes_subscribers_map[res.inputs[4]],
                     self.sync_changes_subscribers_map[res.inputs[5]]], self.time_synchronizer_buffer_size)
            if req.inputs.length() == 7:
                self.time_synchronizer = message_filters.TimeSynchronizer(
                    [self.sync_changes_subscribers_map[res.inputs[0]],
                     self.sync_changes_subscribers_map[res.inputs[1]],
                     self.sync_changes_subscribers_map[res.inputs[2]],
                     self.sync_changes_subscribers_map[res.inputs[3]],
                     self.sync_changes_subscribers_map[res.inputs[4]],
                     self.sync_changes_subscribers_map[res.inputs[5]],
                     self.sync_changes_subscribers_map[res.inputs[6]]], self.time_synchronizer_buffer_size)
            if req.inputs.length() == 8:
                self.time_synchronizer = message_filters.TimeSynchronizer(
                    [self.sync_changes_subscribers_map[res.inputs[0]],
                     self.sync_changes_subscribers_map[res.inputs[1]],
                     self.sync_changes_subscribers_map[res.inputs[2]],
                     self.sync_changes_subscribers_map[res.inputs[3]],
                     self.sync_changes_subscribers_map[res.inputs[4]],
                     self.sync_changes_subscribers_map[res.inputs[5]],
                     self.sync_changes_subscribers_map[res.inputs[6]],
                     self.sync_changes_subscribers_map[res.inputs[7]]], self.time_synchronizer_buffer_size)
        onReconfigure(new_input_worlds)
        self.connection_status = ConnectionStatus.CONNECTED
        if self.ever_connected = False:
            self.ever_connected = True
        self.connection_mutex.release()
        return(True)

    def reconfigureInputs(self, req):
        """
        This method is called when a reconfigure service is received.

        @typedef req: ListRequest
        @param req: The reconfigure request
        """
        rospy.logdebug(
            "[%s] Service ~reconfigure_inputs requested",
            self.node_name)
        try:
            reconfigure(req.inputs)
        except Exception e:
            rospy.logerr("[%s] Error occured : %s", self.node_name, e)
            return(False, e)
        return(True, "")

    def applyChanges(changes_list):
        """
        This method is called when a changes in a world are received.

        @typedef changes_list: ChangesInContextStampedConstPtr
        @param changes_list: The message list received
        """
        for changes_in_ctxt in changes_list:
            invalidations = worlds()[changes_in_ctxt.ctxt.world].applyChanges(changes_in_ctxt.header, changes_in_ctxt.changes)
            onChanges(changes_in_ctxt.ctxt.world, changes_in_ctxt.header, invalidations)

    @abstractmethod
    def onReconfigure(self, worlds):
        """
        This method is called at the end of the reconfigure process.
        Add additional initialization in this method.

        @typedef worlds: string list
        @param worlds: The new input worlds
        """
        raise NotImplementedError

    @abstractmethod
    def onSubscribeChanges(self, world):
        """
        This method is called when a world changes is subscribed by this nodelet.
        Set up additional subscribers or class in this method.

        @typedef world: string
        @param world The world subscribed
        """
        raise NotImplementedError

    @abstractmethod
    def onUnsubscribeChanges(self, worlds):
        """
        This method is called when a world changes is unsubscribed by this nodelet.
        Shut down additional subscribers or class in this method.

        @typedef world: string
        @param world The world unsubscribed
        """
        raise NotImplementedError

    @abstractmethod
    def onChanges(self, world, header, invalidations):
        """
        This method is called when there is a change in a world,
        subclass need to implement this method.
        """
        raise NotImplementedError


    def changesCallback(self, msg):
        """
        This method is called when a changes in a world are received.

        @typedef msg0: ChangesInContextStampedConstPtr
        @param msg0: The 1st message received
        @typedef msg1: ChangesInContextStampedConstPtr
        @param msg1: The 2nd message received
        """
        changes_list = []
        changes_list.append(msg)
        applyChanges(changes_list)

    def changesCallback(self, msg0, msg1):
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
        applyChanges(changes_list)

    def changesCallback(msg0, msg1, msg2):
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
        applyChanges(changes_list)

    def changesCallback(msg0, msg1, msg2, msg3):
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
        applyChanges(changes_list)
        raise NotImplementedError

    def changesCallback(msg0, msg1, msg2, msg3, msg4):
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
        applyChanges(changes_list)

    def changesCallback(msg0, msg1, msg2, msg3, msg4):
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
        applyChanges(changes_list)

    def changesCallback(msg0, msg1, msg2, msg3, msg4, msg5):
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
        applyChanges(changes_list)

    def changesCallback(msg0, msg1, msg2, msg3, msg4, msg5, mgs6):
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
        applyChanges(changes_list)

    def changesCallback(msg0, msg1, msg2, msg3, msg4, msg5, mgs6, msg7):
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
        applyChanges(changes_list)

    def addChangesSubscriber(world):
        """
        This method is called to subscribe to the given world changes.

        @typedef world: string
        @param world: The world to subscribe
        """
        self.addInputWorld(world)
        added = False
        if self.syncronized:
            if world not in self.sync_changes_subscribers_map:
                sync_changes_subscribers_map[world] = \
                    message_filters.Subscriber(
                    world+"/changes", ChangesInContextStamped)
                added = True
        else:
            if world not in self.changes_subscriber_map:
                self.changes_subscriber_map[world] = \
                    rospy.Subscriber(world+"/changes", ChangesInContextStamped)
                added = True
        if added:
            rospy.logdebug("Remove changes subscriber for world <%s>", world)

    def removeChangesSubscriber(world):
        """
        This method is called to remove the given world changes subscriber.

        @typedef world: string
        @param world: The world to unsubscribe
        """
        removed = False
        if self.syncronized:
            if world in self.sync_changes_subscribers_map:
                del self.sync_changes_subscribers_map[world]
                removed = True
        else:
            if world not in self.changes_subscriber_map:
                del self.changes_subscriber_map[world]
                removed = True
        if removed:
            rospy.logdebug("Remove changes subscriber for world <%s>", world)
