from scenario import BoxScenario
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def test_divide_subspace():
    lower = np.array([0,0])
    upper = np.array([6,6])
    num_divisions = [12,3]
    output = BoxScenario.divide_space(lower, upper, num_divisions)
    sorted_output = sorted(output, key=lambda x: x[0]) # sort by start
    plt.figure()
    ax = plt.gca()
    plt.xlim((lower[0] - 1, upper[0] + 1))
    plt.ylim((lower[1] - 1, upper[1] + 1))
    for (start, end) in sorted_output:
        rect = Rectangle(start,end[0] - start[0],end[1] - start[1],linewidth=1,edgecolor='r', facecolor='none')
        ax.add_patch(rect)
    plt.show()

    lower = np.array([0,0,0])
    upper = np.array([6,6,10])
    num_divisions = [2,3,0]
    output = BoxScenario.divide_space(lower, upper, num_divisions)
    sorted_output = sorted(output, key=lambda x: x[0]) # sort by start
    print(sorted_output)

if __name__ == "__main__":
    test_divide_subspace()


