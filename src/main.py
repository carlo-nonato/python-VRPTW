import numpy as np
from timeit import timeit
import argparse as ap

from VRPTW import Customer, VRPTW
from ESPPRC import ESPPRC
from DSSR_ESPPRC import DSSR_ESPPRC
from SSR_SPPRC import SSR_SPPRC

results = []

def test(input_file, espprc_cls):
    global results
    vrptw = VRPTW.from_file(input_file)
    vrptw.vehicles = 100
    vrptw.init_model()
    vrptw.set_espprc_solver(espprc_cls)
    results = vrptw.solve()

if __name__ == "__main__":
    parser = ap.ArgumentParser(description='Solve VRPTW problem.')
    parser.add_argument('input_file', type=str)
    choices = ('exact', 'ssr', 'dssr')
    solvers = (ESPPRC, SSR_SPPRC, DSSR_ESPPRC)
    choice_to_solvers = {choice: cls for choice, cls in zip(choices, solvers)}
    parser.add_argument('-s', dest='espprc_solver', choices=choices,
                        default='exact', help='Specify ESPPRC solver')
    args = parser.parse_args()
    
    espprc_cls = choice_to_solvers[args.espprc_solver]
    print(timeit("test(args.input_file, espprc_cls)", number=1,
                 globals=globals()))
    obj, paths = results
    print(obj)
    for path in paths:
        print(path)