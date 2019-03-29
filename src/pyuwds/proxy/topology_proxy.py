from pyuwds.types.topology import Topology
from uwds_msgs.srv import GetTopology
from proxy import DataProxy

class GetTopologyProxy (DataProxy):
    
    def __init__(self, client, topology):
        #TODO call super
        self.__world_name = ""


    def save_data_from_remote(self, get_topology_srv):
        if get_topology_srv.response.success:
            self.data().reset(get_topology_srv.response.worlds, get_topology_srv.response.clients, get_topology_srv.response.client_interactions)
            return True
        return False

    def _fill_request(self):
        return GetTopology()

class TopologyProxy:

    def __init__(self, client):
        self.__topology = Topology()
        self.__get_topology_proxy = GetTopologyProxy(client, self.__topology)

    def get_topology_from_remote(self):
        return self.__get_topology_proxy.get_data_from_remote()

    def topology(self):
        return self.__topology