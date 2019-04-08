from proxy import ServiceProxy
from uwds_msgs.msg import Invalidations
from uwds_msgs.srv import QueryInContextRequest, QueryInContext
from pyuwds.types.scene import Scene
import rospy

class QueryKnowledgeBaseProxy(ServiceProxy):

    def __init__(self, client, world_name):
        super(QueryKnowledgeBaseProxy, self).__init__(client, 'uwds/query_knowledge_base', QueryInContext)
        self.__world_name = world_name

    def _fill_request(self, query):
        query_srv = QueryInContextRequest()
        query_srv.ctxt.client = self.client
        query_srv.ctxt.world = self.__world_name
        query_srv.query = query
        return query_srv


class KnowledgeBaseProxy(object):

    def __init__(self, client, world_name, meshes_proxy):
        self.__query_proxy = QueryKnowledgeBaseProxy(client, world_name)

    def query_knowledge_base(self, query):
        res = self.__query_proxy.call(query)
        if res is not None:
            if res.success is True:
                return res.result
            else:
                rospy.logerr("[%s::knowledge] Exception occured when processing '%s' query" % (self.__query_proxy.client.name, query))
        return []
