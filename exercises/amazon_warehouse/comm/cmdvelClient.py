import sys
# import Ice
#from .ice.cmdvelIceClient import CMDVelIceClient
from .ros.publisherCmdVel import PublisherCMDVel
from .tools import server2int


def __getPublisherCMDVel(jdrc, prefix):
    '''
    Returns a CMDVel ROS Publisher. This function should never be used. Use getCMDVelClient instead of this

    @param jdrc: Comm Communicator
    @param prefix: Name of client in config file

    @type jdrc: Comm Communicator
    @type prefix: String

    @return CMDVel ROS Publisher

    '''

    '''
    Returns a CMDVel ROS Publisher. This function should never be used. Use getCMDVelClient instead of this

    @param jdrc: Comm Communicator
    @param prefix: Name of client in config file

    @type jdrc: Comm Communicator
    @type prefix: String

    @return CMDVel ROS Publisher

    '''
    if (sys.version_info[0] == 2):
        print("Publishing "+  prefix + " with ROS messages")
        topic = jdrc.getConfig().getProperty(prefix+".Topic")

        client = PublisherCMDVel(topic, jdrc)
        return client
    else:
        print(prefix + ": ROS msg are diabled for python "+ sys.version_info[0])
        return None

def __CMDVeldisabled(jdrc, prefix):
    '''
    Prints a warning that the client is disabled. This function should never be used. Use getCMDVelClient instead of this

    @param jdrc: Comm Communicator
    @param prefix: Name of client in config file

    @type jdrc: Comm Communicator
    @type prefix: String

    @return None

    '''
    print(prefix + " Disabled")
    return None

def getCMDVelClient (jdrc, prefix):
    '''
    Returns a CMDVel Client.

    @param jdrc: Comm Communicator
    @param prefix: Name of client in config file
    @param node: ROS node

    @type jdrc: Comm Communicator
    @type prefix: String
    @type node: ROS node

    @return None if CMDVel is disabled

    '''
    server = jdrc.getConfig().getProperty(prefix+".Server")
    server = server2int(server)

    cons = [__CMDVeldisabled, __getCMDVelIceClient, __getPublisherCMDVel]

    return cons[server](jdrc, prefix)
