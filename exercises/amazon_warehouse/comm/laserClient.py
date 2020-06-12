import sys
import os
from .tools import server2int

# import Ice

rosversion = os.environ["ROS_VERSION"]
server = int(rosversion)

if (server == 2):
    import rclpy

if (server == 1):
    import rospy
    from .ros.listenerLaser import ListenerLaser

# from .ice.laserIceClient import LaserIceClient
from .tools import server2int


def __getLaserClient(jdrc, prefix):
    """
    Returns a Camera ROS2 Subscriber

    """
    # if (server == 2):
    print("Receiving " + prefix + " Image from ROS2 messages")
    topic = jdrc.getConfig().getProperty(prefix + ".Topic")
    client = ListenerCameraros2(topic, jdrc.noderos2)
    return client
    # else:
    #     print(prefix + ": ROS2 msg are disabled for python " + sys.version_info[0])
    #     return None


def __getListenerLaser(jdrc, prefix):
    '''
    Returns a Laser ROS Subscriber. This function should never be used. Use getLaserClient instead of this

    @param jdrc: Comm Communicator
    @param prefix: name of client in config file

    @type jdrc: Comm Communicator
    @type prefix: String

    @return Laser ROS Subscriber

    '''
    print("Receiving " + prefix + "  LaserData from ROS messages")
    topic = jdrc.getConfig().getProperty(prefix + ".Topic")
    client = ListenerLaser(topic)
    return client


def __Laserdisabled(jdrc, prefix):
    '''
    Prints a warning that the client is disabled. This function should never be used. Use getLaserClient instead of this

    @param jdrc: Comm Communicator
    @param prefix: name of client in config file

    @type jdrc: Comm Communicator
    @type prefix: String

    @return None

    '''
    print(prefix + " Disabled")
    return None


def getLaserClient(jdrc, prefix):
    '''
    Returns a Laser Client.

    @param jdrc: Comm Communicator
    @param prefix: name of client in config file

    @type jdrc: Comm Communicator
    @type name: String

    @return None if Laser is disabled

    '''
    server = jdrc.getConfig().getProperty(prefix + ".Server")
    server = server2int(server)

    cons = [__Laserdisabled, __getLaserClient, __getListenerLaser]

    return cons[server](jdrc, prefix)
