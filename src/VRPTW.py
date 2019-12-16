from heapq import heappush, heappop
import numpy as np
import scipy.spatial.distance as sp

class Customer:
    """A customer is a node of the graph. It has an index, a location and
       resource requests.

       A customers stores a set of labels which are used to identify every
       feasible path that reaches it (see 'Labels' class for more information).
       
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
    def __init__(self, index, coords, demand, time_window):
        self.index = index
        self.coords = coords
        self.demand = demand
        self.time_window = time_window
        self.labels = []

    def __lt__(self, other):
        return self.index < other.index

    def __index__(self):
        return self.index

    def __hash__(self):
        return self.index

    def filter_dominated_by(self, label):
        """Removes labels dominated by 'label' from this customer."""

        # The following list comprehension can be faster but label._is_dominated
        # can't be set (maybe it can be moved inside label.dominates()?)
        # self.labels[:] = [lbl for lbl in self.labels
        #                   if not label.dominates(lbl)]
        labels = []
        for lbl in self.labels:
            if label.dominates(lbl):
                # lbl can be already in the 'to_be_extended' queue
                # so we need to signal that this label is no more extendable
                lbl._is_dominated = True
            else:
                labels.append(lbl)
        self.labels = labels

class Label:
    """A label describes a path from the starting node to a customer and the
       resources used to reach this customer.

       Labels are associated with a customer and are used to identify each
       feasible state in which that customer can be reached.

       A dominance relation between labels is needed (see 'dominates' method for
       more information) so only 'best' labels can be kept on a node and the
       resource-minimum path can be found. This is not to be confused with
       the precedence relation between labels needed only for the priority
       queue sorting.

       Note that the unreachable customers set is a so called 'dummy resource'.
       It contains also the already visited customers (they cannot be reached
       again because we are looking for an elementary path).
       
       Attributes:
           customer: the customer whom this label is associated with
           cost: the accumulated cost
           load: the accumulated load
           time: the time spent
           prev: the previous label (used for path reconstruction)
    """

    def __init__(self, customer, cost, load, time, prev=None):
        self.customer = customer
        self.cost = cost
        self.load = load
        self.time = time
        self.unreachable_cs = set([customer])
        self.prev = prev
        self._is_dominated = False

    def __lt__(self, other):
        return self.customer < other.customer

    def dominates(self, label):
        """A label is called 'dominant' when compared to another label, if
           it uses less resources than the other label, meaning that all of
           its resources must be less than or equal to the other's, but there
           must be at least one resource that is different.

           Note that having the unreachable customers set as a resource means
           that a label uses less of this resource if it possesses a subset of
           the other's unreachable customers set.

           Arguments:
               label: the label to compare with

           Returns:
               True if this label dominates 'label', False otherwise
        """

        # since no label can be equal to another (it would mean the same path
        # would be getting followed) maybe we can return the if condition 
        # ( <= for each attribute) so L1 dominates L2 even if they are equal
        if (self.cost <= label.cost and self.load <= label.load
                and self.time <= label.time
                and self.unreachable_cs.issubset(label.unreachable_cs)):
            return (self.cost != label.cost or self.load != label.load
                    or self.time != label.time 
                    or (len(self.unreachable_cs) != len(label.unreachable_cs)))
        
        return False

    def is_dominated(self):
        """Returns True if this label is dominated by any of the labels
           associated with the same customer, False otherwise."""

        for label in self.customer.labels:
            if label.dominates(self):
                return True
        return False

class VRPTW:
    """The problem instance class.

       It stores all the data of the instance and is able to find the exact 
       solution of the VRPTW subproblem also known as Elementary Shortest Path
       Problem with Resource Constraints (ESPPRC). This solution is used in a
       column generation approach to solve the VRPTW.
       
       Updating the dual variables list changes the cost of each arc so it's
       possible to retrieve a different solution of the subproblem.

       Note that the cost of going from customer i to customer j is equal to
       the distance between the two. Also time from i to j is equal to distance.

       Arguments:
           vehicles: number of vehicles available
           capacity: maximum capacity of each vehicle
           customers: a list of Customer objects
    """
    
    def __init__(self, vehicles, capacity, customers):
        self.vehicles = vehicles
        self.capacity = capacity
        # skip starting customer because we don't need paths that visit it
        self.customers = customers[1:]
        # use all the customers because we need distances between eache couple
        self.costs = self.compute_costs(customers)
        self.costs[0, -1] = 1000
        self.costs[-1, 0] = 1000
        self.times = self.costs
        self.duals = np.zeros(len(customers))

    def subproblem(self):
        l0 = Label(c0, 0, 0, 0)
        c0.labels.append(l0)
        to_be_extended = [l0] # priority queue
        while to_be_extended:
            from_label = heappop(to_be_extended)
            # if a label becomes dominated after being pushed in the priority
            # queue, label._is_dominated becomes true and it can be skipped
            if from_label._is_dominated:
                continue

            for to_cus in self.customers:
                if to_cus in from_label.unreachable_cs:
                    continue

                to_label = self.extended_label(from_label, to_cus)
                # needed if the problem is unfeasible and because the last
                # node is not directly reachable from start (costs[0, n] = INF).
                # maybe a better solution can be found
                if not to_label:
                    continue
                
                if to_label.is_dominated():
                    continue

                to_cus.filter_dominated_by(to_label)

                if (self.update_unreachable_cs(to_label)
                        and to_label.is_dominated()):
                    continue

                to_cus.labels.append(to_label)
                heappush(to_be_extended, to_label)

    def extended_label(self, from_label, to_cus):
        """Returns a new label that extends 'from_label' and goes to 'to_cus'.
           
           Arguments:
               from_label: the label to start from
               to_cus: the customer to reach

           Returns:
               A new label with updated resources or None if some resource
               exceeds its limits.
        """

        from_cus = from_label.customer
        cost = (from_label.cost + self.costs[from_cus, to_cus]
                - self.duals[from_cus])
        
        load = from_label.load + to_cus.demand
        if load > self.capacity:
            return
        
        time = max(from_label.time + self.times[from_cus, to_cus],
                   to_cus.time_window[0])
        if time > to_cus.time_window[1]:
            return

        label = Label(to_cus, cost, load, time, from_label)
        label.unreachable_cs.update(from_label.unreachable_cs)
        return label

    def update_unreachable_cs(self, from_label):
        """Updates the unreachable customers set of the 'from_label' label by
           searching for unfeasible labels extending it.
           
           Arguments:
               from_label: the label that needs to be updated
           
           Returns:
               True if the label is updated, False otherwise
        """

        updated = False
        for to_cus in self.customers:
            if to_cus in from_label.unreachable_cs:
                continue

            label = self.extended_label(from_label, to_cus)
            if not label:
                from_label.unreachable_cs.add(to_cus)
                updated = True
        return updated

    @staticmethod
    def from_file(file_name):
        # to be implemented
        pass

    @staticmethod
    def compute_costs(customers):
        coords = [customer.coords for customer in customers]
        return sp.squareform(sp.pdist(coords))

def print_path(label):
    print(label.customer.index, end="")
    if label.prev:
        print(" -> ", end="")
        print_path(label.prev)

# Example
vehicles = 2
capacity = 50
c0 = Customer(0, np.array([0, 0]), 0,  (0, 20)) # A
c1 = Customer(1, np.array([0, 1]), 20, (5, 6))  # B
c2 = Customer(2, np.array([2, 0]), 20, (1, 3))  # C
c3 = Customer(3, np.array([3, 3]), 30, (6, 11)) # D
c4 = Customer(4, np.array([0, 0]), 0,  (0, 20)) # A
# A -> B = 1
# A -> C = 2
# A -> D = 4.24
# B -> C = 2.24
# B -> D = 3.61
# C -> D = 3.16
customers = [c0, c1, c2, c3, c4]
vrptw = VRPTW(vehicles, capacity, customers)
vrptw.duals[1] = 20
vrptw.duals[2] = 10