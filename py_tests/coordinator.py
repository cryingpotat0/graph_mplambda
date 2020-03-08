from pq import RabbitMQQueue
import argparse, subprocess, os, signal, time

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
        self._lambdas = []

    def init_lambdas(self, ):
        self._worker_queue.purge_queue()
        self._graph_queue.purge_queue()

        for i in range(self._num_lambdas):
            p = subprocess.Popen([
                'python', 'lambda.py',
                '--scenario_name', self._scenario_name,
                '--scenario_args', str(self._scenario_args),
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
        pass
        #self._worker_queue.put("test {}".format(self.curr_ind))
        #self.curr_ind += 1
        #time.sleep(0.01)

    def merge_graph(self, ):
        pass

    def find_path(self, ):
        return False

    def solve(self, ):
        self.divide_work()
        self.merge_graph()
        solved = self.find_path() # do djikstras if you can connect goal node
        return solved

if __name__ == "__main__":
    coordinator = LocalCoordinator(
            worker_queue="worker_queue",
            graph_queue="graph_queue",
            scenario_name="BoxScenario", 
            #scenario_args="test",
            scenario_args={
                "start": None,
                "goal": None,
                "robot_radius": None,
                "obstacles": None
                },
            num_lambdas=4
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



