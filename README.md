# Parallelizing Motion Planning Using Lambda Serverless Computing

### Details
- ICRA Paper: https://drive.google.com/file/d/1VHzHlAMv2MfTbqtcT6fy6qaqXoQTjVyG/view
- Video explaining the algorithm and results: https://www.youtube.com/watch?v=yqvL2lCwBCo


### Usage instructions
1. This repository heavily borrows from [previous work](https://github.com/BerkeleyAutomation/mplambda/). Please follow setup and installation instructions from there for packages and libraries.
2. Copy the `scripts/run_experiments.py` file into your build folder and run it. `python run_experiments.py --help` should give configuration options.


### Repository Structure
1. `include/` contains most of the code and logic for this project in header files since the bulk of it is written using C++ templates. 
2. Communication is handled by `comm.hpp`, `packet.hpp`, `buffer.hpp`, `connection.hpp`. These establish a connection to a centralized coordinator that invokes the lambda functions (or for the local case, forks processes). 
3. `include/demo` contains the various testing scenarios. Scenarios have to implement a few methods like `randomSample`, `isValid(state)`, `isValid(start, goal)` in order to be correctly used by the planner.
4. `coordinator.hpp` contains most of the code for the coordinator, and `demo/local_lambda_fixed_graph.hpp` contains most of the code for the lambdas.
