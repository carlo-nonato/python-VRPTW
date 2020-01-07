import numpy as np
from timeit import timeit
import argparse as ap

from parsers import solomon_parse
from VRPTW import Customer, VRPTW

results = []

def test(input_file):
    global results
    vrptw = solomon_parse(input_file)
    vrptw.vehicles = 100
    vrptw.init_model()
    results = vrptw.solve()

if __name__ == "__main__":
    parser = ap.ArgumentParser(description='Process some integers.')
    parser.add_argument('input_file', type=str)
    args = parser.parse_args()

    print(timeit("test(args.input_file)", number=1, globals=globals()))

    obj, paths = results
    print(obj)
    for path in paths:
        print(path)