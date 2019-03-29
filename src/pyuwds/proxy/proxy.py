import rospy
from uwds_msgs.msg import Invalidations

class ServiceProxy(object):
    
    def __init__(self, client, service_name, service_msg):
        self.client = client
        self.service_name = service_name
        self.__service_client = rospy.ServiceProxy(service_name, service_msg)

    def call(self, *param):
        rospy.wait_for_service(self.service_name)
        service_request = self._fill_request(*param)
        try:
            service_response = self.__service_client(service_request)
            if not service_response.success:
                rospy.logerr("[%s::serviceProxy] Error occurred while calling '%s' service" % (self.client.name, self.service_name))
            return service_response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return None
        

    def _fill_request(self, *param):
        raise NotImplementedError
    
class DataProxy (ServiceProxy):

    def __init__(self, client, service_name, data, service_msg):
        super(DataProxy, self).__init__(client, service_name, service_msg)
        self.data = data

    def get_data_from_remote(self, *param):
        print 'get data'
        returns = Invalidations()
        '''
        try:
            print 'trying'
            returns = self._save_data_from_remote(self.call(*param))
        except Exception as e:
            print 'exeception!!'
            rospy.logerr("[%s::dataProxy] Error occured when saving '%s' data into local data-structure: %s" % (self.client.name, self.service_name, e.message))
        '''
        returns = self._save_data_from_remote(self.call(*param))
        return returns

    def _save_data_from_remote(self, *param):
        raise NotImplementedError