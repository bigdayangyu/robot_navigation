import math
import random
import numpy as np  
import matplotlib.pyplot as plt 

class RRT:
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
    
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, start, goal, obstacle_list, map_boundary, sample_rate = 5, nums_iter = 500, expand_dis=1.0, path_resolution=0.05):
        '''
        start, goal = start/goal position
        obstacle_list = list of obstacles
        map_boundary = boundary of search area
        '''
        self.start = self.Node(start[0], start[1])
        self.goal = self.Node(goal[0], goal[1])
        self.obstacle_list = obstacle_list
        self.sample_rate = sample_rate
        self.nums_iteration = nums_iter
        #only sample within map boundary
        self.min_rand = map_boundary[0]
        self.max_rand = map_boundary[1]

        self.expand_dis = expand_dis
        self.path_resolution = path_resolution


        self.node_list = [] # store nodes

    def plan(self):
        self.node_list = [self.start]
        for i in range(self.nums_iteration):
            random_node = self.random_position()
            nearest_index = self.nearest_neighbor(random_node, self.node_list)
            nearest_node = self.node_list[nearest_index]
            #?????
            new_node = self.steer(nearest_node, random_node, self.expand_dis)
            print('new node from steer: ', new_node.x, len(new_node.path_y))

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
            random_node = self.Node(rand_x, rand_y)
        else:
            random_node = self.Node(self.goal.x, self.goal.y)
        return random_node

    def nearest_neighbor(self, rand_node, node_list):
        '''get the index of nearest neighbor
        '''
        distance = [(rand_node.x - node.x)**2 + (rand_node.y - node.y)**2 for node in node_list]
        min_index = distance.index(min(distance))
        return min_index

    # def steer(self, rand_node, node_near):
    #   pass

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
        path = [[self.goal.x, self.goal.y]]
        goal_node = self.node_list[goal_index]
        while goal_node.parent is not None:
            path.append([goal_node.x, goal_node.y])
            goal_node = goal_node.parent
        path.append([goal_node.x, goal_node.y])

        return path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_dist_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_dist_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)

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
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)


def main(gx=6.0, gy=10.0):
    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1)
    ]  # [x, y, radius]
    # Set Initial parameters
    rrt = RRT(start=[0, 0],
              goal=[gx, gy],
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
            # rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.001)  # Need for Mac
            plt.show()



if __name__ == '__main__':
    main()