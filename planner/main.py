import math
import random
import numpy as np  
import matplotlib.pyplot as plt 
from flat_traject import *
from rrt import *
from tracking import *
from plot_car import *


def plot_map(obstacle_list):
        
        for (ox, oy, size) in obstacle_list:
            plot_circle(ox, oy, size)
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        

def plot_circle(x, y, size, color="-b"):  # pragma: no cover
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
    plt.plot(xl, yl, color)
    plt.fill(xl, yl, color)

def main(gx=6.0, gy=8.0):
    print("start " + __file__)

    # ====Setup obstacles====
    obstacleList = [
        (6, 5, 1),
        (5, 5, 1),
        (3, 6, 2),
        # (3, 8, 2),
        # (3, 10, 2),
        # (7, 5, 2),
        # (9, 5, 2),
        # (10, 5, 2),
        # (8, 10, 1)
    ]  
    
    # ====Search Path with RRT====
    rrt = RRT(start=[2, 0],
              goal=[gx, gy],
              steer_angle = 0.0,
              obstacle_list=obstacleList,
              map_boundary=[-2, 15]
              )
    path = rrt.plan()
    
    
    # ====Plot RRT====    
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
                plt.pause(0.00001)

            plt.show()
    
    # ====Trajectory Tracking====
    xs_exe = []
    ys_exe = []
    thetas_ext = []
    for i in range(len(path)-1):
        if i==0:
            node1 = [2,0,0]
        else:
            p = path[-i]
            node1 = [p[0].x,p[0].y,p[0].theta]
        p = path[-i-1]
        node2 = [p[0].x,p[0].y,p[0].theta]
        #print(node1,node2)
        traj = Trajectory(node1, node2, 1.5,1.)  #Kept the same as that in RRT  
        
        node1.append(1.5)
        tout = np.linspace(0,1,1000)
        k = [1,1]
        model = Kinematics(k,traj)
        z = model.trajectory(node1, tout)
        x_num = z[:,0]
        y_num = z[:,1]
        #node2.append(0.1)
        xs_exe.extend(x_num)
        ys_exe.extend(y_num)
        thetas_ext.extend(z[:,2])

    plt.subplots()

    plot_car = Plot_car();
    
    x_new = [xs_exe[0]]
    y_new = [xs_exe[0]]

    for i in range(len(xs_exe)): 

        plt.cla()


        plot_map(obstacleList)
        plt.plot(xs_exe, ys_exe, "-r")

        plot_car.plot_vehicle(xs_exe[i], ys_exe[i], thetas_ext[i])
  
        plt.plot(x_new, y_new, ".")
        if i %500:
            x_new.append(xs_exe[i])
            y_new.append(ys_exe[i])

        plt.grid(True)
        plt.pause(0.0000001)
    
    #print(xs_exe)
    # plt.figure()
    #rrt.draw_graph()
    # plot_map(obstacleList)
    plt.plot(xs_exe,ys_exe,'-r')   
    plt.show()

if __name__ == '__main__':
    main()