#!/usr/bin/env python3
 
import rospy
import numpy as np
import math
from numpy.random import choice
from numpy.random import random_sample
from numpy.random import normal
from numpy.random import uniform
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import matplotlib.mlab as mlab
from std_msgs.msg import Float64MultiArray


class LiveGraph:

    def __init__(self):
        rospy.init_node('live_graph') 
        self.data = [0,0,0]
        rospy.Subscriber("/graph_data", Float64MultiArray , self.data_received)

        fig = plt.figure()
        ani = animation.FuncAnimation(fig, LiveGraph.animate,
            interval=1000, fargs=(self,))
        plt.show()

    def data_received(self, msg):
        self.data = msg.data
      
    def get_data(self):
        return self.data

    @staticmethod
    def animate(i, args):
        graph = args
        data = graph.get_data()
        print(data)
        plt.clf()
        plt.hist(data, density=True, bins=50)


if __name__ == '__main__':
    live_graph = LiveGraph()
    rospy.spin()



# particles = np.arange(2000)
# def read_from_txt():
#     data = open('resample_prob.txt','r').read()
#     lines = data.split('\n')
#     lines = lines[:-1]

#     return lines

# def animate(i):
#     data = read_from_txt()
#     as_float = np.asarray(data, dtype=np.float64, order='C')
#     sorted_vals = np.sort(as_float)
#     ax1.clear()

#     ax1.plot(particles,sorted_vals)


# fig = plt.figure()
# ax1 = fig.add_subplot(1,1,1)
# data = read_from_txt()
# as_float = np.asarray(data, dtype=np.float64, order='C')
# sorted_vals = np.sort(as_float)
# ax1.clear()

# ax1.plot(particles,sorted_vals)

# ani = animation.FuncAnimation(fig, animate, interval=1000)
# ax1.plot(particles,particles)
# plt.show()