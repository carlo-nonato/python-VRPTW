from problem import Problem
from customer import Customer

def GHparse(filename):
    with open(filename) as input_file:
        lines = input_file.readlines()
        K, Q = [int(i) for i in lines[4].split()]
        data = [[int(i) for i in line.split()] for line in lines[9:]]
    for i in range(6):
        P=Problem(K, Q, data[i][1],data[i][2])
        C=Customer(data[i][4:6],data[i][3])
        print (data[i][4:6])
    temp = vars(P)
    temps=vars(C)
    for item in temp:
        print (item , ' : ' , temp[item])
    for items in temps:
        print (items , ' : ' , temps[items])
