from parsers import solomon_parse

if __name__ == "__main__":
    vrptw = solomon_parse("R101.25.txt")
    while vrptw.master_problem():
        pass
