from uwds_msgs.msg import Invalidations
from uwds_msgs.srv import GetTimeline, GetTimelineRequest
from proxy import DataProxy
from pyuwds.types.timeline import Timeline

class GetTimeLineProxy(DataProxy):

    def __init__(self, client, world_name, timeline):
        super(GetTimeLineProxy, self).__init__(client, 'uwds/get_timeline', timeline, GetTimeline)
        self.__world_name = world_name

    def _save_data_from_remote(self, get_timeline_response):
        inv = Invalidations()
        if get_timeline_response.success:
            inv.situation_ids_deleted = self.data.reset(get_timeline_response.origin.data)
            inv.situation_ids_updated = self.data.update(get_timeline_response.situations)
        else:
            #TODO: error ?
            pass
        return inv

    def _fill_request(self):
        get_timeline_request = GetTimelineRequest()
        get_timeline_request.ctxt.client = self.client
        get_timeline_request.ctxt.world = self.__world_name
        return get_timeline_request

class TimelineProxy(object):
    
    def __init__(self, client, world_name):
        self.__timeline = Timeline()
        self.__get_timeline_proxy = GetTimeLineProxy(client, world_name, self.__timeline)

    def get_timeline_from_remote(self):
        return self.__get_timeline_proxy.get_data_from_remote()

    def timeline(self):
        return self.__timeline
