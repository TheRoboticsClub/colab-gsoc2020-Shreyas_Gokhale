import sys
import os

rosversion = os.environ["ROS_VERSION"]
server = int(rosversion)

if ( server == 2):
    import rclpy

if ( server == 1):
    import rospy


def __getListenerIR(jdrc, prefix):
    '''
    Returns a Laser ROS Subscriber. This function should never be used. Use getLaserClient instead of this

    @param jdrc: Comm Communicator
    @param prefix: name of client in config file

    @type jdrc: Comm Communicator
    @type prefix: String

    @return Laser ROS Subscriber

    '''    
    print(prefix + ": This Interface doesn't support ROS msg")
    return None

def __IRdisabled(jdrc, prefix):
    '''
    Prints a warning that the client is disabled. This function should never be used. Use getLaserClient instead of this

    @param jdrc: Comm Communicator
    @param prefix: name of client in config file

    @type jdrc: Comm Communicator
    @type prefix: String

    @return None

    '''
    print( prefix + " Disabled")
    return None

def getIRClient (jdrc, prefix):
    '''
    Returns a Laser Client.

    @param jdrc: Comm Communicator
    @param prefix: name of client in config file

    @type jdrc: Comm Communicator
    @type name: String

    @return None if Laser is disabled

    '''
    server = jdrc.getConfig().getProperty(prefix+".Server")
    if not server:
        server=0

    cons = [__IRdisabled, __getIRIceClient, __getListenerIR]

    return cons[server](jdrc, prefix)