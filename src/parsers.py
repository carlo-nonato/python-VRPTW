def GHparse(filename):
    with open(filename) as input_file:
        lines = input_file.readlines()
        K, Q = [int(i) for i in lines[4].split()]
        data = [[int(i) for i in line.split()] for line in lines[9:]]

    return K, Q, data
