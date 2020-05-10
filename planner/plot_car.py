import matplotlib.pyplot as plt
import numpy as np
# import ode_solver
from random import random


dt = 0.1
show_animation = True
# Vehicle parameters
SCALE = 4;
LENGTH = 4/SCALE # [m]
WIDTH = 3.0/SCALE  # [m]
BACKTOWHEEL = 1.0/SCALE # [m]
WHEEL_LEN = 0.3/SCALE  # [m]
WHEEL_WIDTH = 0.2/SCALE  # [m]
TREAD = 0.7/SCALE  #[m]
WB = 2.5/SCALE  # [m]

class Plot_car:

    def plot_vehicle(self, x, y, theta, steer = 0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover
        outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                            [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

        fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                             [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])

        rr_wheel = np.copy(fr_wheel)

        fl_wheel = np.copy(fr_wheel)
        fl_wheel[1, :] *= -1
        rl_wheel = np.copy(rr_wheel)
        rl_wheel[1, :] *= -1

        Rot1 = np.array([[np.cos(theta), np.sin(theta)],
                         [-np.sin(theta), np.cos(theta)]])
        Rot2 = np.array([[np.cos(steer), np.sin(steer)],
                         [-np.sin(steer), np.cos(steer)]])

        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += WB
        fl_wheel[0, :] += WB

        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T

        outline = (outline.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T

        outline[0, :] += x
        outline[1, :] += y
        fr_wheel[0, :] += x
        fr_wheel[1, :] += y
        rr_wheel[0, :] += x
        rr_wheel[1, :] += y
        fl_wheel[0, :] += x
        fl_wheel[1, :] += y
        rl_wheel[0, :] += x
        rl_wheel[1, :] += y

        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), truckcolor)
        plt.plot(np.array(fr_wheel[0, :]).flatten(),
                 np.array(fr_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(rr_wheel[0, :]).flatten(),
                 np.array(rr_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(fl_wheel[0, :]).flatten(),
                 np.array(fl_wheel[1, :]).flatten(), truckcolor)
        plt.plot(np.array(rl_wheel[0, :]).flatten(),
                 np.array(rl_wheel[1, :]).flatten(), truckcolor)
        plt.plot(x, y, "*")

        # for stopping simulation with the esc key.
        # plt.gcf().canvas.mpl_connect('key_release_event',
        #         lambda event: [exit(0) if event.key == 'escape' else None])

        # plt.xlim(-5, 5)
        # plt.ylim(-16,4)

        # plt.pause(dt)

    def transformation_matrix(self, x, y, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta), x],
            [np.sin(theta), np.cos(theta), y],
            [0, 0, 1]
        ])


    def plot_ellipse(self, xCenter, cBest, cMin, etheta):  # pragma: no cover

        a = np.sqrt(cBest ** 2 - cMin ** 2) / 2.0
        b = cBest / 2.0
        angle = np.pi / 2.0 - etheta
        cx = xCenter[0]
        cy = xCenter[1]

        t = np.arange(0, 2 * np.pi + 0.1, 0.1)
        x = [a * np.cos(it) for it in t]
        y = [b * np.sin(it) for it in t]
        R = np.array([[np.cos(angle), np.sin(angle)],
                      [-np.sin(angle), np.cos(angle)]])
        fx = R * np.array([x, y])
        px = np.array(fx[0, :] + cx).flatten()
        py = np.array(fx[1, :] + cy).flatten()
        plt.plot(cx, cy, "xc")
        plt.plot(px, py, "--c")

# def main():
#     x = [5, 6, 6.4, 8.5, 10.5, 16]
#     y = [5.4, 5.4, 5.4, 5.4, 5.4, 6]
#     theta = [0.1, 0.2, 0.2, 0.2,0.1, -0.6]
#     xCenter = [1,2,3,4,5,6,7]
#     cBest = 1
#     cMin = 0.2
#     etheta = 0.01
#     x_traj = 5.5
#     y_traj = 5.5
#     for i in range(len(x)):

#         plot_vehicle(x[i], y[i], theta[i])
#         # plot_ellipse(xCenter, cBest, cMin, etheta)
# def main2():
#     tout = np.linspace(0,10,100)
#     plot_car = Plot_car();

#     x0 = [-4,1]
#     k = [1,1]
#     model = ode_solver.Kinematics(k)

#     z = model.trajectory(x0, tout)
#     x=  z[:,0]
#     y = z[:,1]
#     theta = 0.01*x
#     print(x.shape)
#     for i in range(len(x)):

#         plot_car.plot_vehicle(x[i], y[i], theta[i])

# if __name__ == '__main__':
#     main2()
