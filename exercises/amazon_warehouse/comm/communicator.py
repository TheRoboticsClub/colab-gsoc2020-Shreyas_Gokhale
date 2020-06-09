import sys
import os
import pprint
import subprocess

rosversion = os.environ["ROS_VERSION"]
server = int(rosversion)

if ( server == 2):
    import rclpy
    from .ros2.listenerCameraros2 import ListenerCameraros2

if ( server == 1):
    import rospy
    from .ros.listenerCamera import ListenerCamera

from .laserClient import getLaserClient
from .cameraClient import getCameraClient
from .pose3dClient import getPose3dClient
from .motorsClient import getMotorsClient
from .rgbdClient import getRgbdClient
from .ardroneextraClient import getArDroneExtraClient
from .navdataClient import getNavdataClient
from .cmdvelClient import getCMDVelClient
from .ptMotorsClient import getPTMotorsClient
from .bumperClient import getBumperClient
from .sonarClient import getSonarClient
from .irClient import getIRClient

import threading



class Communicator:
	'''
    Comm Communicator class

    '''
	def __init__ (self, config, prefix):
		'''
	    Communicator constructor

	    @param config: configuration of communicator
	    
	    @type config: dict

	    '''
		rosserver = False
		ros2server = False

		self.__node = None
		self.noderos2 = None
		self.config = config


		self.__state = ""
		self.__lock = threading.Lock()

		ymlNode = self.config.getProperty(prefix)
		for i in ymlNode:
			if type(ymlNode[i]) is dict and ymlNode[i]["Server"] == 1:
				ros2server = True
			if type(ymlNode[i]) is dict and ymlNode[i]["Server"] == 2:
				rosserver = True

		if rosserver:
			self.__node = rospy.init_node(ymlNode["NodeName"], anonymous=True)
		if ros2server:
			rclpy.init()
			self.noderos2  = rclpy.create_node(ymlNode["NodeName"])

	def destroy (self):
		'''
	    Destroys ROS or ROS2 Node if it is necessary.

	    '''
		if self.__node:
			rospy.signal_shutdown("Node Closed")
		if self.noderos2:
			rclpy.shutdown()

	def getNode(self):
		return self.__node

	def getnoderos2(self):
		return self.noderos2

	def getConfig(self):	
		return self.config 

	def getState(self):
		self.__lock.acquire()
		s = self.__state
		self.__lock.release()
		return s

	def setState (self, state):
		self.__lock.acquire()
		self.__state = state
		self.__lock.release()


	def getCameraClient(self, name):
		'''
	    Returns a Camera client with the configration indicated by the name

	    @param name: name of the client in the config
	    
	    @type name: String

	    '''
		return getCameraClient(self, name)

	def getMotorsClient(self, name):
		'''
	    Returns a Motors client with the configration indicated by the name

	    @param name: name of the client in the config
	    
	    @type name: String

	    '''
		return getMotorsClient(self, name)

	def getPose3dClient(self, name):
		'''
	    Returns a Pose3D client with the configration indicated by the name

	    @param name: name of the client in the config
	    
	    @type name: String

	    '''
		return getPose3dClient(self, name)

	def getLaserClient(self, name):
		'''
	    Returns a Laser client with the configration indicated by the name

	    @param name: name of the client in the config
	    
	    @type name: String

	    '''
		return getLaserClient(self, name)


	def getRgbdClient(self, name):
		'''
	    Returns a RGBD client with the configration indicated by the name

	    @param name: name of the client in the config
	    
	    @type name: String

	    '''
		return getRgbdClient(self, name)

	def getCMDVelClient(self, name):
		'''
	    Returns a CMDVel client with the configration indicated by the name

	    @param name: name of the client in the config
	    
	    @type name: String

	    '''
		return getCMDVelClient(self, name)

	def getNavdataClient(self, name):
		'''
	    Returns a Navdata client with the configration indicated by the name

	    @param name: name of the client in the config
	    
	    @type name: String

	    '''
		return getNavdataClient(self, name)

	def getArDroneExtraClient(self, name):
		'''
	    Returns a ArDroneExtra client with the configration indicated by the name

	    @param name: name of the client in the config
	    
	    @type name: String

	    '''
		return getArDroneExtraClient(self, name)
	def getPTMotorsClient(self, name):
		'''
	    Returns a PTMotors client with the configration indicated by the name

	    @param name: name of the client in the config
	    
	    @type name: String

	    '''
		return getPTMotorsClient(self, name)

	def getBumperClient(self, name):
		'''
	    Returns a Bumper client with the configration indicated by the name

	    @param name: name of the client in the config
	    
	    @type name: String

	    '''
		return getBumperClient(self, name)

	def getSonarClient(self, name):
		'''
	    Returns a Sonar client with the configration indicated by the name

	    @param name: name of the client in the config
	    
	    @type name: String

	    '''
		return getSonarClient(self, name)

	def getIRClient(self, name):
		'''
	    Returns a IR client with the configration indicated by the name

	    @param name: name of the client in the config
	    
	    @type name: String

	    '''
		return getIRClient(self, name)

