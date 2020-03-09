from pq import RabbitMQQueue, WorkMessage
from scenario import SCENARIOS
from logger import Logger
from kdtree import KDTree
import numpy as np
import argparse, json
parser = argparse.ArgumentParser()
parser.add_argument('--type', help='type of lambda')
parser.add_argument('--scenario_name', type=str)
parser.add_argument('--scenario_args', type=str)
parser.add_argument('--worker_queue', type=str)
parser.add_argument('--graph_queue', type=str)
parser.add_argument('--lambda_id', type=int)

class Lambda():
    def __init__(self, ):
        raise NotImplementedError

class LocalLambda():
    def __init__(self, lambda_id, worker_queue, graph_queue, scenario_name, scenario_args, ):
        self._scenario = SCENARIOS[scenario_name](**scenario_args)
        self._lambda_id = lambda_id
        self._worker_queue = RabbitMQQueue(worker_queue)
        self._graph_queue = RabbitMQQueue(graph_queue)
        self._logger = Logger()

        self._logger.log("Started lambda {}".format(lambda_id))

    def do_work(self, ):
        work = self._worker_queue.get()
        if not work:
            return
        work = WorkMessage.deserialize(work.decode('utf-8'))
        self._scenario.ulim = work.ulim
        self._scenario.llim = work.llim
        self._logger.log("Lambda {} doing work on llim: {}, ulim: {}".format(self._lambda_id, work.llim, work.ulim))

        graph = work.networkx_graph
        existing_nodes = list(graph.nodes)
        collision_free_samples = self._scenario.sample()
        all_nodes = collision_free_samples + existing_nodes
        # TODO: get existing samples from work.adjacency list
        points_kdtree = KDTree(all_nodes)
        for (i, sample) in enumerate(collision_free_samples):
            sample = np.array(sample)
            graph.add_node(tuple(sample))
            index, dists = points_kdtree.search(sample, k=10)
            index, dists = index[1:], dists[1:]
            for ind in index:
                other_point = np.array(all_nodes[ind])
                if not self._scenario.is_collision(sample, other_point):
                    graph.add_edge(
                            tuple(sample),
                            tuple(other_point)
                            )

        #import ipdb; ipdb.set_trace()
        response_message = WorkMessage(
                llim=work.llim, 
                ulim=work.ulim, 
                networkx_graph=graph, 
                lambda_id=self._lambda_id
                )
        self._graph_queue.put(response_message.serialize())

        #import ipdb; ipdb.set_trace()
        #    log_str = "lambda {} : {}".format(self._lambda_id, work)
        #    self._logger.log(log_str)

    def shutdown(self, ):
        self._logger.log("Lambda {} graceful shutdown".format(self._lambda_id))
        self._logger.shutdown()

if __name__ == "__main__":
    args = parser.parse_args()
    if args.type == "local":
        lambda_ = LocalLambda(
                lambda_id=args.lambda_id,
                worker_queue=args.worker_queue,
                graph_queue=args.graph_queue,
                scenario_name=args.scenario_name,
                scenario_args=json.loads(args.scenario_args)
                )
    while True:
        try:
            lambda_.do_work()
        except KeyboardInterrupt:
            lambda_.shutdown()
            break
        # TODO: handle shutdown

