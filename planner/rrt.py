import math
import random
import numpy as np  
import matplotlib.pyplot as plt 
from flat_traject import *

class RRT:
    class Node:
        def __init__(self, x, y, theta):
            self.x = x
            self.y = y
            self.theta = theta
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, start, goal, steer_angle, obstacle_list, map_boundary, sample_rate = 5, nums_iter = 500, expand_dis=1.0, linear_vel=1.5,T = 1.):
        '''
        start, goal = start/goal position
        obstacle_list = list of obstacles
        map_boundary = boundary of search area
        '''
        self.start = self.Node(start[0], start[1], steer_angle)
        self.goal = self.Node(goal[0], goal[1], steer_angle)
        self.obstacle_list = obstacle_list
        self.sample_rate = sample_rate
        self.nums_iteration = nums_iter
        #only sample within map boundary
        self.min_rand = map_boundary[0]
        self.max_rand = map_boundary[1]

        self.expand_dis = expand_dis
        self.linear_vel = linear_vel
        self.T = T
        self.L = 1.# length of the car


        self.node_list = [] # store nodes

    def plan(self):
        self.node_list = [self.start]
        for i in range(self.nums_iteration):
            random_node = self.random_position()
            nearest_index = self.nearest_neighbor(random_node, self.node_list)
            nearest_node = self.node_list[nearest_index]
            #?????
            new_node = self.steer(nearest_node, random_node, self.expand_dis)
            # print('new_node: ', new_node.path_x)
            if self.check_colisiton(new_node, self.obstacle_list):
                self.node_list.append(new_node)

            if i % 5 == 0:
                self.draw_graph(random_node)

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.goal, self.expand_dis)
            
                if self.check_colisiton(final_node, self.obstacle_list):
                    print('find_path reached ! ')
                    return self.find_path(len(self.node_list)-1)
            if i % 5:
                self.draw_graph(random_node)
        print('no path found')
        return None

    def random_position(self):
        if random.randint(0,100)>self.sample_rate:
            rand_x = random.uniform(self.min_rand, self.max_rand)
            rand_y = random.uniform(self.min_rand, self.max_rand)
            rand_theta = random.uniform(- 0.05, 0.05)
            random_node = self.Node(rand_x, rand_y, rand_theta)
        else:
            random_node = self.Node(self.goal.x, self.goal.y, self.goal.theta)
        return random_node
        
    def distance_metric(self, rand_node,node_list):
        '''
        Distance metric
        '''
        distance = []
        w = [0.001,2.]        
        for node in node_list:
             
            start = [node.x, node.y, node.theta]
            goal = [rand_node.x, rand_node.y, rand_node.theta]
            traj = Trajectory(start, goal, velocity = self.linear_vel,T=1.)
            path = traj.get_desired_traj()
            path_u = path.path_u_norm
            path_x = np.array(path.path_x)
            path_y = np.array(path.path_y)
            dist_u = np.sum(path_u,axis=0)*w[0]
            
            dist_x = (path_x[1:100] - path_x[0:99])**2
            dist_y = (path_y[1:100] - path_y[0:99])**2
            dist_euclidean = np.sum(dist_x+dist_y,axis=0)*w[1]
            #print(dist_u[0],dist_euclidean)         
            dist = dist_u[0] + dist_euclidean
                        
            distance.append(dist)
            
        return distance
        
    def nearest_neighbor(self, rand_node, node_list):
        '''get the index of nearest neighbor
        '''
        
        distance = self.distance_metric(rand_node,node_list)#[(rand_node.x - node.x)**2 + (rand_node.y - node.y)**2 for node in node_list]
        # theta = math.atan2(rand_node.y - node.y, rand_node.x - node.x)
        min_index = distance.index(min(distance))
        return min_index
        


    def calc_dist_angle(self, start_node, end_node):
        dx = end_node.x - start_node.x
        dy = end_node.y - start_node.y
        distance = np.sqrt(dx*dx + dy*dy)
        theta = math.atan2(dy, dx)
        return distance, theta  

    def calc_dist_to_goal(self, x, y):
        dx = x - self.goal.x
        dy = y - self.goal.y
        return math.hypot(dx, dy)

    def check_colisiton(self, node, obstacle_list):
        '''
        check colision:
            if colides: return False
            else: return True
        '''
        if node is None: 
            return False
        # check each node in path for all obstacles
        for (x_center, y_center, r) in obstacle_list:
            distance_list = []

            for x, y in zip(node.path_x, node.path_y):
                dx = x_center - x
                dy = y_center - y
                distance = dx**2 + dy**2
                distance_list.append(distance)

            if min(distance_list) <= r**2:
                return False
        return True

    def insert_node(self, new_node):
        self.node_list.append(new_node)

    def find_path(self, goal_index):
        # path = [[self.goal]]
        
        goal_node = self.node_list[goal_index]
        path = [[goal_node]]
        while goal_node.parent is not None:
            path.append([goal_node])
            goal_node = goal_node.parent
        # path.append([goal_node])

        return path


    def steer(self, from_node, to_node, extend_length=float("inf")):
        start = [from_node.x, from_node.y, from_node.theta]
        goal = [to_node.x, to_node.y, to_node.theta]
        new_node = self.Node(from_node.x, from_node.y, from_node.theta)
        d, theta = self.calc_dist_angle(new_node, to_node)

        traj = Trajectory(start, goal, velocity = self.linear_vel,T=self.T)
        new_node= traj.get_desired_traj()
        new_node.parent = from_node

        return new_node

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.001)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)


def main(gx=10.0, gy=9.0):
    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [
        (6, 5, 1),
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (10, 5, 2),
        (8, 10, 1)
    ]  # [x, y, radius]
    # Set Initial parameters
    rrt = RRT(start=[2, 0],
              goal=[gx, gy],
              steer_angle = 0.0,
              obstacle_list=obstacleList,
              map_boundary=[-2, 15]
              )
    path = rrt.plan()

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        show_animation = True
        # Draw final path

        if show_animation:
            x = []
            y = []
            for i in range(len(path)):
                p =path[i]
                x.append([p[0].path_x])
                y.append([p[0].path_y])
            
            rrt.draw_graph()
            x_flat =[item for sublist in x for item in sublist]
            y_flat = [item for sublist in y for item in sublist]


            for xd,yd in zip(x_flat, y_flat):
                plt.plot(xd, yd , '-r')
                plt.pause(0.001)

            plt.show()



if __name__ == '__main__':
    main()