
from proxy import ServiceProxy
from scene_proxy import SceneProxy
from timeline_proxy import TimelineProxy
from knowledge_base_proxy import KnowledgeBaseProxy

from uwds_msgs.msg import Client, Invalidations, ChangesInContextStamped, Connection
from uwds_msgs.srv import AdvertiseConnection, AdvertiseConnectionRequest
from std_msgs.msg import Header
import rospy


READ = Connection.READ
WRITE = Connection.WRITE

ConnectionTypeNames = {READ: "read", WRITE: "write"}

class AdvertiseConnectionProxy(ServiceProxy):

    def __init__(self, client, world_name):
        super(AdvertiseConnectionProxy, self).__init__(client, 'uwds/advertise_connection', AdvertiseConnection)
        self.__world_name = world_name

    def _fill_request(self, connection_type, action):
        advertise_connection_srv = AdvertiseConnectionRequest()
        advertise_connection_srv.connection.ctxt.client = self.client
        advertise_connection_srv.connection.ctxt.world = self.__world_name
        advertise_connection_srv.connection.type = connection_type
        advertise_connection_srv.connection.action = action
        return advertise_connection_srv

class WorldProxy(object):

    def __init__(self, client, meshes_proxy, world_name):
        self.__client = client
        self.__world_name = world_name
        self.__global_frame_id = ""
        self.__meshes_proxy = meshes_proxy
        self.__scene_proxy = SceneProxy(client, world_name, meshes_proxy)
        self.__timeline_proxy = TimelineProxy(client, world_name)
        self.__knowledge_base_proxy = KnowledgeBaseProxy(client, world_name)
        self.__advertise_connection_proxy = AdvertiseConnectionProxy(client, world_name)
        self.__ever_connected = False
        self.__ever_send_changes = False
        self.__scene_proxy.get_scene_from_remote()
        self.__timeline_proxy.get_timeline_from_remote()


        self.__changes_subscriber = rospy.Subscriber(world_name + '/changes', ChangesInContextStamped, self.changes_callback, queue_size=20)
        self.__changes_publisher = rospy.Publisher(world_name + '/changes', ChangesInContextStamped, queue_size=20)


    def meshes(self):
        return self.__meshes_proxy.meshes()

    def scene(self):
        return self.__scene_proxy.scene()

    def timeline(self):
        return self.__timeline_proxy.timeline()

    def __getitem__(self, query):
        return self.__knowledge_base_proxy.query_knowledge_base(query)

    def connect(self, callback):
        if not self.__ever_send_changes:
            if not self.__ever_connected:
                self.advertise_connection_to_remote(Connection.READ, Connection.CONNECT)
                self.__ever_connected = True
            self.__on_changes = callback
            return True
        return False

    def changes_callback(self, msg):
        inv = Invalidations()
        inv.node_ids_deleted = self.scene().remove(msg.changes.nodes_to_delete)
        inv.node_ids_updated = self.scene().update(msg.changes.nodes_to_update)

        inv.situation_ids_updated = self.timeline().update(msg.changes.situations_to_update)

        for sit in self.timeline().situations():
            if sit.end.data != rospy.Time(0):
                msg.changes.situations_to_delete.append(sit.id)

        inv.situation_ids_deleted = self.timeline().remove(msg.changes.situations_to_delete)

        inv.mesh_ids_deleted = msg.changes.meshes_to_delete
        self.meshes().remove(msg.changes.meshes_to_delete)
        u = self.meshes().update(msg.changes.meshes_to_update)
        inv.mesh_ids_updated = u
        if self.__ever_connected:
            self.__on_changes(self.__world_name, msg.header, inv)

    def update(self, changes, header=None):
        if len(changes.nodes_to_update) > 0 or len(changes.situations_to_update) or len(changes.nodes_to_delete) > 0 or len(changes.situations_to_delete) > 0:
            if header is None:
                header = Header(stamp=rospy.Time.now(), frame_id=self.__global_frame_id)
            if not self.__ever_connected:
                if not self.__ever_send_changes:
                    self.advertise_connection_to_remote(Connection.WRITE, Connection.CONNECT)
                    self.__ever_send_changes = True
                msg = ChangesInContextStamped()
                msg.ctxt.client = self.__client
                msg.ctxt.world = self.__world_name
                msg.header = header
                msg.changes = changes

                # while self.__changes_publisher.get_num_connections() < 1:
                #     rospy.sleep(0.15)
                self.__changes_publisher.publish(msg)
                return True
            else:
                return False
        return False

    def advertise_connection_to_remote(self, connection_type, action):
        advertise_connection_response = self.__advertise_connection_proxy.call(connection_type, action)
        if not advertise_connection_response.success:
            pass
        return advertise_connection_response.success

    def push_mesh_from_3d_file(self, filename, scale=None):
        raise NotImplementedError

    def push_scene_from_3d_file(self, filename):
        raise NotImplementedError
