from concurrent_container import ConcurrentContainer

class Meshes (ConcurrentContainer):

    def update(self, meshes):
        ids = [m.id for m in meshes]
        super(Meshes, self).update(ids, meshes)
        return ids
