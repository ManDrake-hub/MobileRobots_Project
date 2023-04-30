import matplotlib.pyplot as plt
import glob
import pathlib
import json
import numpy as np
from collections import OrderedDict
import math
from typing import List, Tuple


def get_ellipse(center_x, center_y, radius_x, radius_y) -> Tuple[np.ndarray, np.ndarray]:
    t = np.linspace(0, 2*math.pi, 100)
    return center_x+radius_x*np.cos(t), center_y+radius_y*np.sin(t)

def get_cmap(n, name='inferno'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct 
    RGB color; the keyword argument name must be a standard mpl colormap name.'''
    return plt.cm.get_cmap(name, n)


if __name__ == "__main__":
    files = glob.glob(str(pathlib.Path(__file__).parent.resolve()) + "/MonteCarlo/exercise_5/*")

    waypoints = OrderedDict()
    waypoints_real = OrderedDict()

    for file in files:
        with open(file, "r") as f:
            montecarlo = json.load(f)
            for waypoint in montecarlo:
                waypoints_real[waypoint["waypoint"]] = waypoint["pose_real"]

                if waypoint["waypoint"] in waypoints:
                    waypoints[waypoint["waypoint"]] += [waypoint["pose"], ]
                else:
                    waypoints[waypoint["waypoint"]] = [waypoint["pose"], ]

    cmap = get_cmap(len(waypoints))
    
    for waypoint in waypoints:
        plt.scatter(waypoints_real[waypoint][0], 
                    waypoints_real[waypoint][1], s=20, c=cmap(waypoint))

        w = np.array(waypoints[waypoint])
        mean_x=np.mean(w[:, 0])
        mean_y=np.mean(w[:, 1])
        std_x=np.std(w[:, 0])
        std_y=np.std(w[:, 1])

        print("Waypoint: ", waypoint, "mean: ", mean_x, mean_y,
                                      "std: ", std_x, std_y)
        plt.plot(*get_ellipse(mean_x, mean_y, std_x, std_y), label=waypoint, c=cmap(waypoint))
    plt.legend()
    plt.show()