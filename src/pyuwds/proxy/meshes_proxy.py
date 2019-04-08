from proxy import ServiceProxy, DataProxy
from uwds_msgs.srv import PushMesh, PushMeshRequest, GetMesh, GetMeshRequest
from pyuwds.types.meshes import Meshes

class PushMeshProxy(ServiceProxy):

    def __init__(self, client):
        super(PushMeshProxy, self).__init__(client, 'uwds/push_mesh', PushMesh)

    def _fill_request(self, mesh):
        push_mesh_request = PushMeshRequest()
        push_mesh_request.request.mesh = mesh
        return push_mesh_request

class GetMeshProxy(DataProxy):

    def __init__(self, client, meshes):
        super(GetMeshProxy, self).__init__(client, 'uwds/get_mesh', meshes, GetMesh)

    def _save_data_from_remote(self, get_mesh_response):
        if get_mesh_response.success:
            self.data.update([get_mesh_response.mesh])
            return True
        return False

    def _fill_request(self, mesh_id):
        get_mesh_request = GetMeshRequest()
        get_mesh_request.mesh_id = mesh_id
        return get_mesh_request

class MeshesProxy(object):

    def __init__(self, client):
        self.__meshes = Meshes()
        self.__push_mesh_proxy = PushMeshProxy(client)
        self.__get_mesh_proxy = GetMeshProxy(client, self.__meshes)

    def push_mesh_to_remote(self, mesh):
        try:
            push_mesh_srv = self.__push_mesh_proxy.call(mesh)
            return push_mesh_srv.response.success
        except Exception as e:
            print "error!!!!!!!!!!!!!!!!!!!!!!!"

    def get_mesh_from_remote(self, mesh_id):
        return self.__get_mesh_proxy.get_data_from_remote(mesh_id)

    def meshes(self):
        return self.__meshes
