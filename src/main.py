import numpy as np
from timeit import timeit

from parsers import solomon_parse
from VRPTW import Customer, VRPTW

def test():
    vrptw = solomon_parse("R101.25.txt")
    vrptw.vehicles = 100
    vrptw.init_model()
    return vrptw.solve()

if __name__ == "__main__":
    print(timeit(test, number=1))
    obj, paths = test()
    print(obj)
    for path in paths:
        print(path)