from VRPTW import *
import numpy as np

def solomon_parse(filename):
    with open(filename) as input_file:
        lines = input_file.readlines()
        K, Q = [int(i) for i in lines[4].split()]
        data = [[int(i) for i in line.split()] for line in lines[9:]]

    customers = [Customer(line[0], np.array(line[1:3]), line[3],
                          line[4:6], line[6])
                 for line in data if line]
    vrptw = VRPTW(K, Q, customers)
    return vrptw
