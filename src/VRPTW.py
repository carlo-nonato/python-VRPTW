import numpy as np
import scipy.spatial.distance as sp
import gurobipy as gp

from ESPPRC import ESPPRC

class Customer:
    """A customer is a node of the graph. It has an index, a location and
       resource requests.
       
       There is an order relation between customers, namely the lexicographic
       order, so a customer goes before another when its index is less than the
       other's.
       Also, customers can be used directly in indexing data structures.
       
       Attributes:
           index: index used to determine lexicographic order
           coords: the coordinates where the customer is placed
           demand: the amount of resource requested
           time_window: interval of time in which the resource must be delivered
    """

    def __init__(self, index, coords, demand, time_window, service_time):
        self.index = index
        self.coords = coords
        self.demand = demand
        self.time_window = time_window
        self.service_time = service_time

    def __lt__(self, other):
        return self.index < other.index

    def __index__(self):
        return self.index

    def __hash__(self):
        return self.index

    def __repr__(self):
        return f"Customer <{self.index}>"

class VRPTW:
    """The Vehicle Routing Problem with Time Windows instance class.

       It stores all the data of the instance and is able to find the exact 
       solution of the VRPTW problem through a column generation approach.
       A set partitioning linear model tries to solve a reduced problem with an
       initial set of paths. Then by using the dual variables of this master
       problem, it updates the cost of the arcs and generates a new path,
       solving the subproblem, also known as Elementary Shortest Path Problem
       with Resource Constraints (ESPPRC).
       This process is repeated until the reduced cost of the new path becomes
       non-negative.

       Note that the cost of going from customer i to customer j is equal to
       the euclidean distance between the two. Also time from i to j is equal to
       distance.

       Arguments:
           vehicles: number of vehicles available
           capacity: maximum capacity of each vehicle
           customers: a list of Customer objects
    """

    @staticmethod
    def from_file(filename):
        """Create a VRPTW instance from a text file."""

        with open(filename) as input_file:
            lines = input_file.readlines()
            vehicles, capacity = [int(i) for i in lines[4].split()]
            data = [[int(i) for i in line.split()] for line in lines[9:]]

        customers = [Customer(line[0], np.array(line[1:3]), line[3], line[4:6],
                              line[6])
                     for line in data if line]
        vrptw = VRPTW(vehicles, capacity, customers)
        return vrptw

    @staticmethod
    def compute_costs(customers):
        """Returns a square symmetric matrix of distances between customers."""

        coords = [customer.coords for customer in customers]
        return sp.squareform(sp.pdist(coords))

    def __init__(self, vehicles, capacity, customers):
        self.vehicles = vehicles
        self.capacity = capacity
        self.customers = customers
        self.costs = self.compute_costs(customers)
        self.times = self.costs

    def init_model(self):
        """Inits the master problem model."""

        # TODO: the first set of paths must be feasible, now it's only a guess
        path_costs = self.costs[0, 1:]*2
        self.paths = [(0, customer.index, 0) for customer in self.customers[1:]]
        self.paths_set = set(self.paths)
        n = len(self.paths)
        A = np.eye(n)
        b = np.ones(n)
        model = gp.Model("VRPTW")
        model.Params.OutputFlag = 0
        x = model.addMVar(n, name="v", vtype=gp.GRB.CONTINUOUS)
        model.setObjective(path_costs @ x, gp.GRB.MINIMIZE)
        model.addMConstrs(A, x, '=', b)
        model.addConstr(sum(x) <= self.vehicles)
        self.model = model

    def set_espprc_solver(self, espprc_cls):
        self.espprc = espprc_cls(self.capacity, self.customers, self.costs,
                                 self.times)

    def solve(self):
        while True:
            self.model.optimize()
            duals = [constr.Pi for constr in self.model.getConstrs()]
            self.espprc.duals[1:] = duals[:-1]
            labels = self.espprc.solve()
            if labels[0].cost - duals[-1] >= -1e-9:
                return (self.model.getObjective().getValue(), self.used_paths())
            for label in labels:
                if label.cost - duals[-1] >= 0:
                    break
                path = tuple(customer.index for customer in label.path)
                if path in self.paths_set:
                    continue
                cost = label.cost + sum(self.espprc.duals[list(path)])
                self.add_path(path, cost)
    
    def add_path(self, path, cost):
        """Add path to the master problem.
           
           Arguments:
               path: the path to be added
               cost: the path cost
        """

        self.paths.append(path)
        self.paths_set.add(path)
        path_indices = [index - 1 for index in path[1:-1]]
        coeffs = np.zeros(len(self.customers) - 1)
        coeffs[path_indices] = 1
        self.model.addVar(obj=cost, name=f"v{len(self.paths)-1}",
                          column=gp.Column(coeffs,self.model.getConstrs()[:-1]))

    def used_paths(self):
        """Returns the path used in the optimal solution."""
        
        return [(path, var.Obj, var.x)
                for path, var in zip(self.paths, self.model.getVars())
                if var.x != 0]