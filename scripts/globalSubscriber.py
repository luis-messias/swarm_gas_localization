import rospy
from nav_msgs.msg import Odometry
from olfaction_msgs.msg._gas_sensor import gas_sensor
import tf
import numpy as np

class globalSubscriber():

    def __init__(self):
        self.numberRobots = 10 #int(rospy.get_param("/" + self.nodeName + '/num_robots',1))

        self.robotPosition_x = np.zeros(self.numberRobots)
        self.robotPosition_y = np.zeros(self.numberRobots)

        self.x = [[] for i in range(self.numberRobots)] 
        self.y = [[] for i in range(self.numberRobots)] 
        self.t = [[] for i in range(self.numberRobots)] 
        self.raw = [[] for i in range(self.numberRobots)] 

        self.subscriberOdometry = []
        self.subscriberGas = []

        for i in range(self.numberRobots):
            self.subscriberOdometry.append(rospy.Subscriber("/swarm_node/robot_" + str(i) + "/odom", Odometry, self.callbackOdometry, i))
            self.subscriberGas.append(rospy.Subscriber("/Mox" + str(i) + "/Sensor_reading", gas_sensor, self.callbackMox, i))
    
    def callbackOdometry(self, msg, i):
        self.robotPosition_x[i] = msg.pose.pose.position.x
        self.robotPosition_y[i] = msg.pose.pose.position.y

    def callbackMox(self, msg, i):
        self.x[i].append(self.robotPosition_x[i])
        self.y[i].append(self.robotPosition_y[i])
        self.t[i].append(float(msg.header.stamp.secs) + float(msg.header.stamp.nsecs)*(10**-9))
        self.raw[i].append(msg.raw)
        while len(self.x) > 200:
            self.x[i].pop()
            self.y[i].pop()
            self.t[i].pop()
            self.raw[i].pop()

    def getPosition(self):
        return self.robotPosition_x, self.robotPosition_y

    def getArrays(self):
        x = np.concatenate(self.x)
        y = np.concatenate(self.y)
        t = np.concatenate(self.t)
        raw = np.concatenate(self.raw)

        return x, y, t, raw




    

    


