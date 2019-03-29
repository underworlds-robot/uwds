from proxy import DataProxy
from uwds_msgs.msg import Invalidations
from uwds_msgs.srv import GetSceneRequest, GetScene
from pyuwds.types.scene import Scene

class GetSceneProxy(DataProxy):

    def __init__(self, client, world_name, scene, meshes):
        super(GetSceneProxy, self).__init__(client, 'uwds/get_scene', scene, GetScene)
        self.__world_name = world_name
        self.__meshes = meshes

    def _save_data_from_remote(self, get_scene_response):
        inv = Invalidations()
        if get_scene_response.success:
            inv.node_ids_deleted = self.data.reset(get_scene_response.root_id)
            for node in get_scene_response.nodes:
                for property in node.properties:
                    if property.name == 'meshes' and property.data != '':
                        for mesh_id in property.data.split(','):
                            if self.__meshes.get_mesh_from_remote(mesh_id):
                                inv.mesh_ids_updated.append(mesh_id)
                inv.node_ids_updated.append(self.data.update([node]))
        return inv

    def _fill_request(self):
        get_scene_srv = GetSceneRequest()
        get_scene_srv.ctxt.client = self.client
        get_scene_srv.ctxt.world = self.__world_name
        return get_scene_srv
     
class SceneProxy(object):

    def __init__(self, client, world_name, meshes_proxy):
        self.__scene = Scene()
        self.__get_scene_proxy = GetSceneProxy(client, world_name, self.__scene, meshes_proxy)

    def get_scene_from_remote(self):
        return self.__get_scene_proxy.get_data_from_remote()

    def scene(self):
        return self.__scene