#include <coordinator.hpp>
#include <pq.hpp>
#include <jilog.hpp>
#include <string>

mpl::LocalCoordinator::LocalCoordinator(GenericMessageQueue worker_queue, GenericMessageQueue graph_queue, int num_lambdas, std::string scenario_name, std::string scenario_args) {
  this->worker_queue = worker_queue;
  this->graph_queue = graph_queue;
  this->num_lambdas = num_lambdas;
  this->scenario_name = scenario_name;
  this->scenario_args = scenario_args;
}


int main() {
  auto worker_queue = RabbitMQMessageQueue("localhost", 5672, "test");
  worker_queue.put<std::string>("aaaaaaaaaaabcdefghi");
  JI_LOG(INFO) << "output " << worker_queue.get<std::string>();
}
