import math
import random
import matplotlib.pyplot as plt
import numpy as np

class OBST_Map:
    """
    Class for obstacle map
    """
    def __init__(self, obstacle_list):
        self.obstacle_list = obstacle_list

    def plot_map(self):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.show()

    def plot_circle(self, x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)
        plt.fill(xl, yl, color)


def main():
    obstacleList = [
        (5, 5, 1),
        (3, 6, 1),
        (3, 8, 1),
        (3, 10, 1),
        (7, 5, 1),
        (9, 5, 1),
        (8, 10, 1)
    ]  # [x, y, radius]
    maps = OBST_Map(obstacleList)
    maps.plot_map()
    


if __name__ == '__main__':
    main()