from concurrent_container import ConcurrentContainer
from uwds_msgs.msg import Node
from nodes import Nodes
import rospy
import uuid

class Scene:

    def __init__(self):
        self.reset(str(uuid.uuid4()))
    
    def update(self, nodes):
        current_time = rospy.Time.now()
        for node in nodes:
            node.last_update.data = current_time
            if node.parent not in self.__nodes:
                node.parent = self.root_id
        self.__nodes.update(nodes)
        return [n.id for n in nodes if n.name != "root"]

    def remove(self, ids):
        self.__nodes.remove(ids)
        return ids

    def root_id(self):
        return self.__root_id

    def nodes(self):
        return self.__nodes

    def reset(self, root_id):
        self.__root_id = root_id
        if not hasattr(self, '__nodes'):
            node_ids = []
            self.__nodes = Nodes()
        else:
            node_ids = self.__nodes.ids()
            self.__nodes.reset()
        root = Node(id=self.__root_id, name="root")
        root.position.pose.orientation.w = 1.0
        self.__nodes.update([root])
        return node_ids