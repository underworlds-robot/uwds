from world_proxy import WorldProxy

class WorldsProxy(object):

    def __init__(self, client, meshes):
        self.__client = client
        self.__meshes = meshes
        self.__worlds = {}

    def __getitem__(self, world_name):
        if world_name not in self.__worlds.keys():
            self.__worlds[world_name] = WorldProxy(self.__client, self.__meshes, world_name)
        return self.__worlds[world_name]

    def close(self):
        self.__worlds.clear()

    def has(self, world_name):
        return True if world_name in self.__worlds else False
