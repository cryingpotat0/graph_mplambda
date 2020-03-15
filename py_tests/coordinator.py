from pq import RabbitMQQueue, WorkMessage
from scenario import SCENARIOS
import argparse, subprocess, os, signal, time, json
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from kdtree import KDTree

parser = argparse.ArgumentParser()
args = parser.parse_args()

class Coordinator():
    def __init__(self, ):
        raise NotImplementedError

class LocalCoordinator(Coordinator):
    def __init__(self, worker_queue, graph_queue, num_lambdas, scenario_name, scenario_args, ):
        """
        problem: args to define problem for lambda
        """
        self._num_lambdas = num_lambdas
        self._worker_queue_name = worker_queue
        self._graph_queue_name = graph_queue
        self._worker_queue = RabbitMQQueue(worker_queue)
        self._graph_queue = RabbitMQQueue(graph_queue)
        self._scenario_name = scenario_name
        self._scenario_args = scenario_args
        self._scenario = SCENARIOS[scenario_name](**scenario_args)
        self._lambdas = []
        self._work_in_progress = {}
        self._overall_graph = nx.Graph()
        self._graph_by_division = {}
        self.curr_ind = 0

    def init_lambdas(self, ):
        self._worker_queue.purge_queue()
        self._graph_queue.purge_queue()

        for i in range(self._num_lambdas):
            p = subprocess.Popen([
                'python', 'lambda.py',
                '--scenario_name', self._scenario_name,
                '--scenario_args', json.dumps(self._scenario_args),
                '--worker_queue', self._worker_queue_name,
                '--graph_queue', self._graph_queue_name,
                '--lambda_id', str(i),
                '--type', 'local'
                ])
            self._lambdas.append(p)
            #import ipdb; ipdb.set_trace()

    def shutdown(self, ):
        self._worker_queue.purge_queue()
        self._graph_queue.purge_queue()

        for p in self._lambdas:
            os.kill(p.pid, signal.SIGINT)
        self._worker_queue.shutdown()
        self._graph_queue.shutdown()

    def divide_work(self, ):
        """
        Assume a perfect square amount of work for now. Not a scalable way to divide work in this manner.
        """
        if not hasattr(self, 'first_division'):
            self.first_division = True
        if self.first_division:
            self.first_division = False
            problem_dimension = self._scenario._dim
            llim, ulim = self._scenario.llim, self._scenario.ulim
            num_divisions_per_axis = int(np.power(self._num_lambdas, 1 / problem_dimension))
            num_divisions = [num_divisions_per_axis] * problem_dimension
            subspaces = self._scenario.divide_space(llim, ulim, num_divisions)
            self._subspaces = subspaces
            self._overall_graph.add_node(tuple(self._scenario.start))
            self._overall_graph.add_node(tuple(self._scenario.goal))
            for (llim, ulim) in subspaces:
                # get adjacency list within that region and remaining regions
                id = (tuple(llim), tuple(ulim))
                curr_graph = nx.Graph()
                self._graph_by_division[id] = curr_graph
                curr_graph.add_node(tuple(self._scenario.start))
                curr_graph.add_node(tuple(self._scenario.goal))
                curr_message = WorkMessage(llim, ulim, curr_graph)
                self._worker_queue.put(curr_message.serialize())
                
        #else:

        #self._worker_queue.put("test {}".format(self.curr_ind))
        #self.curr_ind += 1
        #time.sleep(0.01)

    def merge_graph(self, ):
        """
        merge_graph and reassign work on queue
        """
        work = self._graph_queue.get()
        if work:
            work = WorkMessage.deserialize(work.decode('utf-8'))
            id = (tuple(work.llim), tuple(work.ulim))
            curr_graph = self._graph_by_division[id]
            curr_graph = nx.compose(curr_graph, work.networkx_graph)

            self._overall_graph = nx.compose(self._overall_graph, work.networkx_graph)
            overall_nodelist = list(self._overall_graph.nodes)
            overall_kdtree = KDTree(overall_nodelist)
            required_vertices = []
            for vertex in self._scenario.get_vertices(work.llim, work.ulim):
                idxs = overall_kdtree.search_in_distance(vertex, 20)
                required_vertices.extend([overall_nodelist[i] for i in idxs])
            graph_to_be_sent = self._overall_graph.subgraph(required_vertices)
            curr_graph = nx.compose(curr_graph, graph_to_be_sent)
            curr_message = WorkMessage(work.llim, work.ulim, curr_graph)
            self._worker_queue.put(curr_message.serialize())
            self._graph_by_division[id] = curr_graph


    def plot2d(self, graph, path=None, show=False):
        plt.figure()
        plt.xlim([self._scenario.llim[0] - 10, self._scenario.ulim[0] + 10])
        plt.ylim([self._scenario.llim[1] - 10, self._scenario.ulim[1] + 10])
        ax = plt.gca()
        for (start, end) in self._subspaces:
            rect = Rectangle(start,end[0] - start[0],end[1] - start[1],linewidth=1,edgecolor='r', facecolor='none')
            ax.add_patch(rect)
        for (start, end) in self._scenario._obstacle_config:
            rect = Rectangle(start,end[0] - start[0],end[1] - start[1],linewidth=1,edgecolor='b', facecolor='none')
            ax.add_patch(rect)

        for node in graph.nodes:
            plt.plot([node[0]], [node[1]], 'go')
        for (node, neighbor) in graph.edges:
            plt.plot([node[0], neighbor[0]], [node[1], neighbor[1]], 'k--')
        if path:
            path_edges = [(path[i], path[i+1]) for i in range(len(path) - 1)]
            for (node, neighbor) in path_edges:
                plt.plot([node[0], neighbor[0]], [node[1], neighbor[1]], 'y-')
        plt.plot([self._scenario.start[0]], [self._scenario.start[1]], '^r')
        plt.plot([self._scenario.goal[0]], [self._scenario.goal[1]], '^c')
        if show: plt.show()
        #plt.plot()


    def find_path(self, ):
        cost = lambda v1, v2, d: np.linalg.norm(np.array(v1) - np.array(v2))
        try:
            path = nx.dijkstra_path(self._overall_graph, tuple(self._scenario.start), tuple(self._scenario.goal), weight=cost) 
        except nx.exception.NetworkXNoPath:
            return False
        return path

    def solve(self, ):
        self.divide_work()
        self.merge_graph()
        solved = self.find_path() # do djikstras if you can connect goal node
        if solved:
            self.plot2d(self._overall_graph, path=solved, show=True)
        return solved

if __name__ == "__main__":
    coordinator = LocalCoordinator(
            worker_queue="worker_queue",
            graph_queue="graph_queue",
            scenario_name="BoxScenario", 
            #scenario_args="test",
            scenario_args={
                "start": [10.55, 10],
                "goal": [80.0, 80.0],
                "robot_radius": 5,
                "llim": [0, 0],
                "ulim": [100, 100],
                "obstacles": [
                        #([0, 0], [70, 70]), # border
                        #([15, 15], [40, 40]), # (bottom_left, top_right)
                        ([20, 60], [60, 70]), # (bottom_left, top_right)
                    ]
                },
            num_lambdas=9
            )
    coordinator.init_lambdas()
    while True:
        try:
            path_found = coordinator.solve()
            if path_found: break
        except KeyboardInterrupt:
            coordinator.shutdown()
            break
        except Exception as e:
            coordinator.shutdown()
            raise e



