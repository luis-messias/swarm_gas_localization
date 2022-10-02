import rospy
import numpy as np
from globalSubscriber import globalSubscriber
from gradientEstimator import gradientEstimator

rospy.init_node("gas_localization_node")

subscriber = globalSubscriber()
estimator = gradientEstimator()

while not rospy.is_shutdown():
    x, y, t, raw = subscriber.getArrays()
    x0, y0 = subscriber.getPosition()

    if len(t) > 0:
        min = np.min(raw)
        indice = np.where(raw == min)[0][0]
        print(indice, min)
        print(x[indice], y[indice], raw[indice])
        if np.min(raw) < -1:
            
            break

        try:
            grad = estimator.gradientEstimator( x, y, t, raw, x0, y0)
            estimator.publishGrad(x0, y0, grad, 0.000005)
        except:
            print("ERRO")

        print(np.min(raw))  

    rospy.sleep(0.5)



