# python-VRPTW
Python implementation of some column generation based algorithms for VRPTW.

*Disclaimer: This is a project made for educational purposes only.*

This program solves the VRPTW (Vehicle Routing Problem with Time Windows) with a column generation based approach and different dynamic programming algorithms for the subproblem, called ESPPRC (Elementary Shortest Path with Resource Constraints).
Until now there are two implementation of the subproblem: the exact dynamic programming and the decremental state space relaxation (DSSR). They are "inspired" (meaning that they are not perfect implementations) respectively by [1] and [2].

The program also apply optionally a branch and bound scheme to the number of vehicles, so that the optimal solution returned has an integer number of vehicles. This is not sufficient to have also integer variables (meaning that every path returned is either used fully or not at all), but it is enough in some cases.

## Dependencies
 - Python
 - Numpy and Scipy
 - Gurobi Optimizer (Python interface)

## Usage
```console
$ main.py [-h] [-s {exact,ssr,dssr}] [--bb] input_file
```
Note: SSR is there only for testing since it's not a method for solving the ESPPRC. In fact it solves the SPPRC which can return cyclic paths. So if you use it, it may enter an endless loop because the optimal solution is cyclic.

## References 
  [1] D. Feillet, P. Dejax, M. Gendreau, C. Gueguen. *An Exact Algorithm for the Elementary Shortest Path Problem with Resource Constraints: Application to Some Vehicle Routing Problems*, 2004.<br/>
  [2] G. Righini, M. Salani. *New Dynamic Programming Algorithms for the Resource-Constrained Elementary Shortest Path Problem*, 2005.
