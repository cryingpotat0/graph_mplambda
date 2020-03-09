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
    NUM_SAMPLES_PER_RUN = 20

    def __init__(self, start, goal, llim, ulim, robot_radius, obstacles, dim=2):
        """
        start:
        obstacles: Only rectangular for now. np.array of (x,y) positions defining a set of obstacles
        """
        self._dim = dim
        start, goal = np.array(start), np.array(goal)
        self._check_last_axis(start, "start")
        self._check_last_axis(goal, "goal")

        self._obstacle_config = obstacles
        self._obstacles = self.create_obstacles(obstacles)
        self._obs_kdtree = KDTree(self._obstacles)
        self._robot_radius = robot_radius
        self.llim = np.array(llim)
        self.ulim = np.array(ulim)
        self.start = start
        self.goal = goal

    def create_obstacles(self, obstacle_config):
        # assume 2d obstacles for now
        obstacle_points = []
        for (start, end) in obstacle_config:
            (start_x, start_y), (end_x, end_y) = start, end
            for i in range(start_x, end_x + 1):
                obstacle_points.append([i, start_y])
                obstacle_points.append([i, end_y])

            for i in range(start_y, end_y + 1):
                obstacle_points.append([start_x, i])
                obstacle_points.append([end_x, i])
        return np.array(obstacle_points)

    def _check_last_axis(self, arr, name=""):
        assert arr.shape[-1] == self._dim, "Invalid input {} for box of dimension {}".format(name, self._dim)

    def point_in_collision(self, point):
        idxs, dist = self._obs_kdtree.search(point)
        if dist <= self._robot_radius:
            return True  # collision
        return False  # OK

    def is_collision(self, start, goal):
        """
        is the path from start to goal in collision
        """
        curr = start.copy()
        diff = goal - start
        d = np.linalg.norm(diff)
        if d >= self.MAX_EDGE_LEN:
            return True
        diff /= d

        nsteps = int(d // self._robot_radius)
        for i in range(nsteps):
            idxs, dist = self._obs_kdtree.search(curr)
            if dist <= self._robot_radius:
                return True # collision
            curr += self._robot_radius * diff

        # goal check
        idxs, dist = self._obs_kdtree.search(goal)
        if dist <= self._robot_radius:
            return True  # collision
        return False  # OK


    @staticmethod
    def get_vertices(llim, ulim):
        # TODO: assume 2 dim for now
        return [
                [llim[0], llim[1]],
                [ulim[0], llim[1]],
                [llim[0], ulim[1]],
                [ulim[0], ulim[1]],
                ]


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

    def sample(self, ):
        samples = np.random.uniform(low=0, high=1, size=(self.NUM_SAMPLES_PER_RUN, self._dim))
        samples = samples * (self.ulim - self.llim) + self.llim
        collision_free = []
        for sample in samples:
            if self.point_in_collision(sample): continue
            collision_free.append(list(sample))
        return collision_free




SCENARIOS = {
        "BoxScenario": BoxScenario
        }

