import pika, json
import numpy as np
import networkx as nx

class Queue():
    def __init__(self, ):
        raise NotImplementedError

    def put(self, item, ):
        raise NotImplementedError

    def get(self, ):
        raise NotImplementedError

class RabbitMQQueue(Queue):
    def __init__(self, queue_name):
        parameters = pika.ConnectionParameters("localhost", )
        connection = pika.BlockingConnection(parameters)
        channel = connection.channel()
        channel.queue_declare(queue=queue_name)

        self._parameters = parameters
        self._connection = connection
        self._channel = channel
        self._queue_name = queue_name

    def put(self, item, ):
        self._channel.basic_publish(
                exchange='',
                routing_key=self._queue_name,
                body=item
                )

    def get(self, ):
        method_frame, header_frame, body = self._channel.basic_get(queue=self._queue_name, )
        if method_frame:
            self._channel.basic_ack(method_frame.delivery_tag)
            return body
        return None

    def shutdown(self, ):
        self._channel.close()
        self._connection.close()

    def purge_queue(self, ):
        self._channel.queue_purge(queue=self._queue_name)

class WorkMessage():
    def __init__(self, llim, ulim, networkx_graph, lambda_id=None):
        self.llim = np.array(llim)
        self.ulim = np.array(ulim)
        self.networkx_graph = networkx_graph
        self.lambda_id = lambda_id

    def serialize(self, ):
        msg_dict = {
                "llim": list(self.llim),
                "ulim": list(self.ulim),
                "graph": nx.readwrite.json_graph.node_link_data(self.networkx_graph),
                }
        if self.lambda_id is not None:
            msg_dict["lambda_id"] = self.lambda_id
        #import ipdb; ipdb.set_trace()
        return json.dumps(msg_dict)

    @classmethod
    def deserialize(cls, msg_dict_json):
        msg_dict = json.loads(msg_dict_json)
        lambda_id = None
        if "lambda_id" in msg_dict:
            lambda_id = msg_dict["lambda_id"]

        return cls(
                llim=msg_dict["llim"], 
                ulim=msg_dict["ulim"], 
                networkx_graph=nx.readwrite.json_graph.node_link_graph(msg_dict["graph"]),
                lambda_id=lambda_id
                )


