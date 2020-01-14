import numpy as np
from timeit import timeit
import argparse as ap

from VRPTW import Customer, VRPTW
from ESPPRC import ESPPRC
from DSSR_ESPPRC import DSSR_ESPPRC
from SSR_SPPRC import SSR_SPPRC

results = []

def test(input_file, espprc_cls, bb):
    global results
    vrptw = VRPTW.from_file(input_file)
    vrptw.init_model()
    vrptw.set_espprc_solver(espprc_cls)
    if bb:
        results = vrptw.bb_solve()
    else:
        results = vrptw.solve()

if __name__ == "__main__":
    parser = ap.ArgumentParser(description='Solve VRPTW problem.')
    parser.add_argument('input_file', type=str)
    choices = ('exact', 'ssr', 'dssr')
    solvers = (ESPPRC, SSR_SPPRC, DSSR_ESPPRC)
    choice_to_solvers = dict(zip(choices, solvers))
    parser.add_argument('-s', dest='espprc_solver', choices=choices,
                        default='exact', help='Specify ESPPRC solver')
    parser.add_argument('--bb', action='store_true', help=('Apply a branch and'
                        'bound scheme on number of vehicles'))
    args = parser.parse_args()
    
    espprc_cls = choice_to_solvers[args.espprc_solver]
    print(timeit("test(args.input_file, espprc_cls, args.bb)", number=1,
                 globals=globals()))
    if results:
        obj, paths = results
        print(obj)
        for path in paths:
            print(path)
    else:
        print("Problem is infeasible.")