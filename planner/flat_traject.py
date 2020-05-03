import math
import random
import numpy as np  
import matplotlib.pyplot as plt 

class Trajectory:
    class Node:
        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta
            self.path_x = []
            self.path_y = []
            self.parent = None


    def __init__(self, start, goal, velocity):
        self.start = self.Node(start[0], start[1], start[2])
        self.goal = self.Node(goal[0], goal[1], goal[2])
        d, tehta = self.calc_dist_angle(self.start, self.goal)
        self.T = float(d/velocity)

        self.vel = velocity
        self.y0 = [self.start.x, self.start.y]
        self.yf = [self.goal.x, self.goal.y]

        self.dy0 = [self.vel*math.cos(self.start.theta),self.vel*math.sin(self.start.theta) ]
        self.dyf = [self.vel*math.cos(self.goal.theta),self.vel*math.sin(self.goal.theta) ]

    def poly3(self, t):
        if isinstance(t, float):
            return np.array([[t**3],[t**2],[t],[1]])
        else:   
            # return np.array([[t**3],[t**2],[t],[np.ones(t.shape)]])
            new_size = t.shape[0]
            a = np.array([t**3]).reshape(1,new_size)
            b  = np.array([t**2]).reshape(1,new_size)
            c = np.array([t]).reshape(1,new_size)
            d = np.ones(t.shape).reshape(1,new_size)
            return np.concatenate((a, b,c,d), axis=0)
    

    def dpoly3(self, t):
        if isinstance(t, float):
            return  np.array([[3*t**2],[2*t],[1],[0]])
        else:
            # return np.array([[3*t**2],[2*t],[np.ones(t.shape)],[np.zeros(t.shape)]])
                        # return np.array([[t**3],[t**2],[t],[np.ones(t.shape)]])
            new_size = t.shape[0]
            a = np.array([3*t**2]).reshape(new_size,1)
            b  = np.array([2*t]).reshape(new_size,1)
            c = np.ones(t.shape).reshape(new_size,1)
            d = np.zeros(t.shape).reshape(new_size,1)
            return np.concatenate((a, b,c,d), axis=1)

    def d2poly3(self, t):
        if isinstance(t, float):
            return np.array([[6*t],[2],[0],[0]])
        else:
            return np.array([[6*t], [2], [np.zeros(t.shape)], [np.zeros(t.shape)]])
            new_size = t.shape[0]
            a = np.array([6*t]).reshape(new_size,1)
            b  = np.array([2]).reshape(new_size,1)
            c = np.zeros(t.shape).reshape(new_size,1)
            d = np.zeros(t.shape).reshape(new_size,1)
            return np.concatenate((a, b,c,d), axis=1)

    def poly3_coeff(self):
        Y = np.array([[self.y0[0], self.dy0[0], self.yf[0], self.dyf[0]],
                      [self.y0[1], self.dy0[1], self.yf[1], self.dyf[1]]])
        L = np.array([self.poly3(0.0), self.dpoly3(0.0), self.poly3(self.T), self.dpoly3(self.T)])
        # print(L.shape)
        L = L.reshape(4,4).T
        A = np.dot(Y,np.linalg.inv(L))
        # print(A)
        return A
    def get_desired_traj(self):

        x = np.dot(self.poly3_coeff(),self.poly3(np.linspace(0.0, self.T, num= 100)))
     
        self.goal.path_x.append(list(x[0,:]))
        self.goal.path_y.append(list(x[1,:]))
        self.goal.path_x = self.goal.path_x[0]
        self.goal.path_y = self.goal.path_y[0]

        self.goal.x = x[0,-1]
        self.goal.y = x[1,-1]
        # print('goal.path_x', self.goal.path_x)
        return self.goal

    # def plot_desired_traj(self):
    #     x = self.get_desired_traj(self.start, self.goal)
    #     print(x)
    #     plt.plot(x[0,:], x[1,:], '-r')
    #     plt.show()

    def calc_dist_angle(self, start_node, end_node):
        dx = end_node.x - start_node.x
        dy = end_node.y - start_node.y
        distance = np.sqrt(dx*dx + dy*dy)
        theta = math.atan2(dy, dx)
        return distance, theta  


# def main():
#     start = [-5,-3,0.5]
#     goal = [0,0,1]
#     T = 10

#     traj = Trajectory(start, goal, 1, T)
#     traj.get_desired_traj()



# if __name__ == '__main__':
#     main()