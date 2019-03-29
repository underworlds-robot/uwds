from concurrent_container import ConcurrentContainer
from uwds_msgs.msg import ClientInteraction, Connection
import rospy

class Topology (ConcurrentContainer):

    def __init__(self):
        self.__clients = ConcurrentContainer()
        self.__client_interactions_by_world = ConcurrentContainer()
    
    def clients(self):
        return self.__clients

    def client_interactions(self):
        return self.__client_interactions_by_world
        
    def client_interactions_by_world(self, world_name):
        return self.__client_interactions_by_world[world_name]
        
    def update(self, ctx, interaction_type, action_type):
        self._lock()
        
        if action_type == Connection.CONNECT:
            self.__clients.update(ctx.client.id, ctx.client)

            if not self.__client_interactions_by_world.has(ctx.world):
                self.__client_interactions_by_world.update(ctx.world, ConcurrentContainer())
                
            interaction_msg = ClientInteraction(ctxt=ctx, type=interaction_type)
            self.client_interactions_by_world(ctx.world).update(interaction_msg.ctxt.client.id, interaction_msg)

        elif action_type == Connection.DISCONNECT:
            self.client_interactions_by_world(ctx.world).remove(ctx.client.id)

        self._unlock()

    def reset(self, worlds=None, clients=None, client_interactions=None):
        self.reset()

        if worlds is not None and clients is not None and client_interactions is not None:
            self._lock()

            for client in clients:
                self.__clients.update(client.id, client)
            
            for client_interaction in client_interactions:
                if not self.__client_interactions_by_world.has(client_interaction.ctxt.world):
                    self.__client_interactions_by_world.update(client_interaction.ctxt.world, ConcurrentContainer())
                self.client_interactions_by_world(client_interaction.ctxt.world).update(client_interaction.ctxt.client.id, client_interaction)
            
            self._unlock()