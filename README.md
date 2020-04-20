# Parallelizing Motion Planning Using Lambda Serverless Computing

### Usage instructions
1. This repository heavily borrows from [previous work](https://github.com/BerkeleyAutomation/mplambda/). Please follow setup and installation instructions from there for packages and libraries.
2. Copy the `scripts/run_experiments.py` file into your build folder and run it. `python run_experiments.py --help` should give configuration options.


### Repository Structure
1. `include/` contains most of the code and logic for this project in header files since the bulk of it is written using C++ templates. 
2. Communication is handled by `comm.hpp`, `packet.hpp`, `buffer.hpp`, `connection.hpp`. These establish a connection to a centralized coordinator that invokes the lambda functions (or for the local case, forks processes). 
3. `include/demo` contains the various testing scenarios. Scenarios have to implement a few methods like `randomSample`, `isValid(state)`, `isValid(start, goal)` in order to be correctly used by the planner.
4. `coordinator.hpp` contains most of the code for the coordinator, and `demo/local_lambda_fixed_graph.hpp` contains most of the code for the lambdas.

### Algorithm Structure
1. The coordinator invokes **k** lambdas as determined by the command line arguments. These 'lambdas' can be either locally spawned using `fork` or spawned on AWS after deploying the lambda function. 
2. Each lambda divides the space into hypercubes based on the `num_divisions` argument and assigns itself a fixed region to sample. For example in a 2 dimensional space, `num_divisions=(1,2)` would create 6 regions in total, with one split along the x-axis and one split along the y-axis.
3. First the lambda samples points within it's space. It then checks if these points are "near" any other lambdas using the [formula of distance from a point to a hypercube](https://math.stackexchange.com/questions/2133217/minimal-distance-to-a-cube-in-2d-and-3d-from-a-point-lying-outside). These points, along with any newly sampled edges and vertices are sent to the coordinator using the previously mentioned communication setup. Finally any points received from other lambdas are connected using the nearest neighbor structure as well.
4. The process is terminated by the coordinator after a fixed amount of time, which then uses functions defined in `demo/mpl_robot.hpp`to connect this graph to the queried starts and goals, perform Djikstras and return the paths. The coordinator also saves the graphs to disk which can be reloaded to facilitate replanning over new starts and goals.

