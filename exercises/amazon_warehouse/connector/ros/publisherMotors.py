import threading

import rospy
from geometry_msgs.msg import Twist

from .threadPublisher import ThreadPublisher


def cmdvel2Twist(vel):
    '''
    CMDVel to ROS Twist. 

    @param vel: JderobotTypes CMDVel to translate

    @type img: JdeRobotTypes.CMDVel

    @return a Twist translated from vel

    '''
    tw = Twist()
    tw.linear.x = vel.vx
    tw.linear.y = vel.vy
    tw.linear.z = vel.vz
    tw.angular.x = vel.ax
    tw.angular.y = vel.ay
    tw.angular.z = vel.az

    return tw



class CMDVel:

    def __init__(self):

        self.vx = 0 # vel in x[m/s] (use this for V in wheeled robots)
        self.vy = 0 # vel in y[m/s]
        self.vz = 0 # vel in z[m/s]
        self.ax = 0 # angular vel in X axis [rad/s]
        self.ay = 0 # angular vel in X axis [rad/s]
        self.az = 0 # angular vel in Z axis [rad/s] (use this for W in wheeled robots)
        self.timeStamp = 0 # Time stamp [s]


    def __str__(self):
        s = "CMDVel: {\n   vx: " + str(self.vx) + "\n   vy: " + str(self.vy)
        s = s + "\n   vz: " + str(self.vz) + "\n   ax: " + str(self.ax) 
        s = s + "\n   ay: " + str(self.ay) + "\n   az: " + str(self.az)
        s = s + "\n   timeStamp: " + str(self.timeStamp)  + "\n}"
        return s 



class PublisherMotors:
    '''
        ROS Motors Publisher. Motors Client to Send CMDVel to ROS nodes.
    '''

    def __init__(self, topic, maxV, maxW):
        '''
        ListenerMotors Constructor.

        @param topic: ROS topic to publish
        
        @type topic: String

        '''
        self.maxW = maxW
        self.maxV = maxV

        self.topic = topic
        self.data = CMDVel()
        self.pub = rospy.Publisher(self.topic, Twist, queue_size=1)
        # rospy.init_node("Amazon")
        self.lock = threading.Lock()

        self.kill_event = threading.Event()
        self.thread = ThreadPublisher(self, self.kill_event)

        self.thread.daemon = True
        self.start()

    def publish(self):
        '''
        Function to publish cmdvel. 
        '''
        self.lock.acquire()
        tw = cmdvel2Twist(self.data)
        self.lock.release()
        self.pub.publish(tw)

    def stop(self):
        '''
        Stops (Unregisters) the client. If client is stopped you can not start again, Threading.Thread raised error

        '''
        self.kill_event.set()
        self.pub.unregister()

    def start(self):
        '''
        Starts (Subscribes) the client. If client is stopped you can not start again, Threading.Thread raised error

        '''
        self.kill_event.clear()
        self.thread.start()

    def getMaxW(self):
        return self.maxW

    def getMaxV(self):
        return self.maxV

    def sendVelocities(self, vel):
        '''
        Sends CMDVel.

        @param vel: CMDVel to publish
        
        @type vel: CMDVel

        '''
        self.lock.acquire()
        self.data = vel
        self.lock.release()

    def sendV(self, v):
        '''
        Sends V velocity. uses self.sendVX

        @param v: V velocity
        
        @type v: float

        '''
        self.sendVX(v)

    def sendL(self, l):
        '''
        Sends L velocity. uses self.sendVY

        @param l: L velocity
        
        @type l: float

        '''
        self.sendVY(l)

    def sendW(self, w):
        '''
        Sends W velocity. uses self.sendAZ

        @param w: W velocity
        
        @type w: float

        '''
        self.sendAZ(w)

    def sendVX(self, vx):
        '''
        Sends VX velocity.

        @param vx: VX velocity
        
        @type vx: float

        '''
        self.lock.acquire()
        self.data.vx = vx
        self.lock.release()

    def sendVY(self, vy):
        '''
        Sends VY velocity.

        @param vy: VY velocity
        
        @type vy: float

        '''
        self.lock.acquire()
        self.data.vy = vy
        self.lock.release()

    def sendAZ(self, az):
        '''
        Sends AZ velocity.

        @param az: AZ velocity
        
        @type az: float

        '''
        self.lock.acquire()
        self.data.az = az
        self.lock.release()
