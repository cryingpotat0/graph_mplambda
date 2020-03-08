import numpy as np
from kdtree import KDTree

class Scenario():
    def __init__(self, ):
        raise NotImplementedError

    def is_collision(self, ):
        raise NotImplementedError

    def divide_space(self, ):
        raise NotImplementedError

class BoxScenario(Scenario):
    """
    Create a euclidean "box structure" scenario with a spherical robot as a test scenario
    """
    MAX_EDGE_LEN = 30.0 # don't try to connect edges further than this

    def __init__(self, start, goal, robot_radius, obstacles, dim=2):
        """
        start:
        obstacles: np.array of (x,y) positions defining a set of obstacles
        """
        self._dim = dim
        self._check_last_axis(start, "start")
        self._check_last_axis(goal, "goal")

        self._obs_kdtree = KDTree(obstacles)
        self._obstacles = obstacles
        self._start = start
        self._goal = goal
        self._robot_radius = robot_radius

    def _check_last_axis(self, arr, name=""):
        assert arr.shape[-1] == self._dim, "Invalid input {} for box of dimension {}".format(name, self._dim)

    def is_collision(self, start, goal):
        curr = start.copy()
        diff = goal - start
        d = np.norm(diff)
        if d >= self.MAX_EDGE_LEN:
            return True
        diff /= d

        nsteps = d // self._robot_radius
        for i in range(nsteps):
            idxs, dist = self._obs_kdtree.search(curr)
            if dist[0] <= self._robot_radius:
                return True # collision
            curr += self._robot_radius * diff

        # goal check
        idxs, dist = self._obs_kdtree.search(goal)
        if dist[0] <= self._robot_radius:
            return True  # collision
        return False  # OK

    @staticmethod
    def divide_space(lower, upper, num_divisions):
        """
        start: np.array (dim, ) - minimum bounds of the space to subdivide
        end: np.array(dim, ) - maximum bounds of the space to subdivide
        num_divisions: np.array(dim, ) - number of divisions along each axis >= 0

        returns: list of (lower, upper) of new subspaces
        """
        
        lower, upper, num_divisions = list(lower), list(upper), list(num_divisions)
        subspaces = [(lower, upper)]
        for i, nd in enumerate(num_divisions):
            new_subspaces = []
            while len(subspaces) > 0:
                curr_lower, curr_upper = subspaces.pop()
                increment = (curr_upper[i] - curr_lower[i]) / (nd + 1)
                for j in range(nd + 1):
                    new_lower = curr_lower[:i] + [curr_lower[i] + increment * j] + curr_lower[i+1:]
                    new_upper = curr_upper[:i] + [curr_lower[i] + increment * (j+1)] + curr_upper[i+1:]
                    new_subspaces.append((new_lower, new_upper))
            subspaces = new_subspaces
        return subspaces

    def sample(self, llim, ulim):
        pass
        #TODO:

        



SCENARIOS = {
        "BoxScenario": BoxScenario
        }

