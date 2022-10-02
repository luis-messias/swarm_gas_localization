import rospy
from geometry_msgs.msg import Pose
import numpy as np
from sklearn.linear_model import LinearRegression

class gradientEstimator():

    def __init__(self):
        self.numberRobots = 10 #int(rospy.get_param("/" + self.nodeName + '/num_robots',1))


        self.goal_x = np.zeros(self.numberRobots)
        self.goal_y = np.zeros(self.numberRobots)
        self.gradients = np.zeros((self.numberRobots,2))

        self.goalPublisher = []

        for i in range(self.numberRobots):
            self.goalPublisher.append(rospy.Publisher("/controller_swarm_node/goal_" + str(i), Pose, queue_size = 2))


    def regression(self, x, y, t, raw, x0, y0, t0):
        X = np.asarray([x,y,t]).T
        Y = np.array(raw)
        sample_weight = np.zeros(x.shape)
        c = 0.35
        #c = 0.2

        for i in range(sample_weight.shape[0]):
            dist =  np.abs(t[i] - t0)**2*100  + np.abs(x[i] - x0)**2 + np.abs(y[i] - y0)**2   
            dist =  np.abs(t[i] - t0)**2  + np.abs(x[i] - x0)**2 + np.abs(y[i] - y0)**2    
 
            sample_weight[i] = c / (c+dist)

        regr = LinearRegression()
        regr.fit(X, Y, sample_weight = sample_weight)

        return regr.coef_, regr.intercept_
    
    def gradientEstimator(self, x, y, t, raw, x0, y0):
        for i in range(self.numberRobots):
            coef_, intercept_ = self.regression( x, y, t, raw, x0[i], y0[i], float(rospy.Time.now().secs) + float(rospy.Time.now().nsecs)*(10**-9))
            self.gradients[i][0] = coef_[0]
            self.gradients[i][1] = coef_[1]

        return (self.gradients)

    def publishGrad(self, x0, y0, grad, learningRate):
        goal = Pose()

        print("Publish: ")

        for i in range(self.numberRobots):
            goal.position.x = x0[i] - grad[i][0] * learningRate
            goal.position.y = y0[i] - grad[i][1] * learningRate

            print("DX: ", - grad[i][0] * learningRate, "DY: ", - grad[i][1] * learningRate)
            self.goalPublisher[i].publish(goal)

        

    
    




    

    


