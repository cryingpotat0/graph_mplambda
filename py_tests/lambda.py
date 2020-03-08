from pq import RabbitMQQueue
from scenario import SCENARIOS
from logger import Logger
import numpy as np
import argparse
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
        #self._scenario = SCENARIOS[scenario_name](**scenario_args)
        self._lambda_id = lambda_id
        self._worker_queue = RabbitMQQueue(worker_queue)
        self._graph_queue = RabbitMQQueue(graph_queue)
        self._logger = Logger()

        self._logger.log("Started lambda {}".format(lambda_id))

    def do_work(self, ):
        work = self._worker_queue.get()
        if work:
            log_str = "lambda {} : {}".format(self._lambda_id, work)
            self._logger.log(log_str)

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
                scenario_args=args.scenario_args
                )
    while True:
        try:
            lambda_.do_work()
        except KeyboardInterrupt:
            lambda_.shutdown()
            break
        # TODO: handle shutdown

