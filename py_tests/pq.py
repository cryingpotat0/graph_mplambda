import pika

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

            


