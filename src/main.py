import numpy as np
from timeit import timeit

from parsers import solomon_parse
from VRPTW import Customer, VRPTW

def test():
    vrptw = solomon_parse("R101.25.txt")
    # vrptw.vehicles = 100
    # vrptw.init_model()
    vrptw.solve()

if __name__ == "__main__":
    # Example
    # vehicles = 2
    # capacity = 50
    # c0 = Customer(0, np.array([0, 0]), 0,  (0, 20), 0) # A
    # c1 = Customer(1, np.array([0, 1]), 20, (5, 6), 0)  # B
    # c2 = Customer(2, np.array([2, 0]), 20, (1, 3), 0)  # C
    # c3 = Customer(3, np.array([3, 3]), 30, (6, 11), 0) # D
    # A -> B = 1
    # A -> C = 2
    # A -> D = 4.24
    # B -> C = 2.24
    # B -> D = 3.61
    # C -> D = 3.16
    # customers = [c0, c1, c2, c3]
    # vrptw = VRPTW(vehicles, capacity, customers)
    # vrptw.duals[1] = 20
    # vrptw.duals[2] = 10

    #print(timeit(test, number=1))
    vrptw = solomon_parse("R101.25.txt")
    vrptw.vehicles = 100
    vrptw.init_model()
    obj, paths = vrptw.solve()
    print(obj)
    for path in paths:
        print(path)