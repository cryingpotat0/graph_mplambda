#include <coordinator.hpp>
#include <pq.hpp>
#include <jilog.hpp>
#include <getopt.h>
#include <string>

mpl::LocalCoordinator::LocalCoordinator(GenericMessageQueue worker_queue, GenericMessageQueue graph_queue, int num_lambdas, std::string scenario_name, std::string scenario_args) {
  this->worker_queue = worker_queue;
  this->graph_queue = graph_queue;
  this->num_lambdas = num_lambdas;
  this->scenario_name = scenario_name;
  this->scenario_args = scenario_args;
}

void mpl::LocalCoordinator::init_lambdas() {

}


int main(int argc, char *argv[]) {
  int send_flag = 0;
  auto worker_queue = RabbitMQMessageQueue("localhost", 5672, "work");
  auto graph_queue = RabbitMQMessageQueue("localhost", 5672, "graph");
  static struct option longopts[] = {
    { "send", no_argument, &send_flag, 1 },
    { NULL, 0, NULL, 0 }
  };

  for (int ch ; (ch = ::getopt_long(argc, argv, "", longopts, NULL)) != -1 ; ) { }
  if (send_flag) {
    std::string test = "teeeeeeessssssstttt";
    worker_queue.put<std::string>(test);
  } else {
    while(1) JI_LOG(INFO) << "output " << worker_queue.get<std::string>();
  }
}
