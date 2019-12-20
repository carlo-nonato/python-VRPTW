from heapq import heappush, heappop
import numpy as np

def print_path(label, end="\n"):
    if label.prev:
        print_path(label.prev,"")
        print(" -> ", end="")
    print(label.customer.index, end=end)

class Label:
    """A label describes a path from the depot to a customer and the resources
       used in this path.
       
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

    @staticmethod
    def depot_label(depot):
        """Creates a Label with no resources and that can be extended to the
           same customer it is associated with."""

        label = Label(depot, 0, 0, 0)
        label.unreachable_cs.clear()
        return label

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

    def __repr__(self):
        return f"{(self.customer, self.cost, self.load, self.time)}"

    @property
    def path(self):
        """Returns the path described by this label."""

        label = self
        path = []
        while label.prev:
            path.append(label.customer.index)
            label = label.prev
        path.append(label.customer.index)
        return list(reversed(path))
	
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

    def filter_dominated(self):
        """Removes labels dominated by this label on its customer."""

        # The following list comprehension can be faster but label._is_dominated
        # can't be set (maybe it can be moved inside label.dominates()?)
        # self.customer.labels[:] = [label for label in self.customer.labels
        #                            if not self.dominates(label)]
        labels = []
        for label in self.customer.labels:
            if self.dominates(label):
                # label can be already in the 'to_be_extended' queue
                # so we need to signal that this label is no more extendable
                label._is_dominated = True
            else:
                labels.append(label)
        self.customer.labels = labels

    def update_unreachable_from_prev(self):
        """Updates unreachable customers set with customers in previous label
           set."""

        if self.prev:
            self.unreachable_cs.update(self.prev.unreachable_cs)

class ESPPRC:
    """The Elementary Shortest Path Problem with Resource Constraints
       instance class.
       
       It stores instance data and is able to solve the ESPPRC problem with an
       exact dynamic programming approach.
       It uses a dual variable array so it can be used in a column generation
       approach for vehicle routing problems.

       Attributes:
           capacity: maximum capacity of each vehicle
           customers: a list of Customer objects
           costs: a matrix of costs of each arc
           times: a matrix of times needed for each arc
    """

    def __init__(self, capacity, customers, costs, times):
        self.capacity = capacity
        self.customers = customers
        self.costs = costs
        self.times = times
        self.duals = np.zeros(len(self.customers))

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
        if to_cus == self.customers[0]:
            label.unreachable_cs.update(self.customers)
        else:
            label.unreachable_cs.update(from_label.unreachable_cs)
        return label

    def solve(self):
        for customer in self.customers:
            customer.labels = []
        depot = self.customers[0]
        to_be_extended = [Label.depot_label(depot)] # priority queue
        while to_be_extended:
            from_label = heappop(to_be_extended)
            # if a label becomes dominated after being pushed in the priority
            # queue, label._is_dominated becomes true and it can be skipped
            if from_label._is_dominated:
                continue

            from_label.update_unreachable_from_prev()
            for to_cus in self.customers:
                if (to_cus in from_label.unreachable_cs
                        or (from_label.customer is depot and to_cus is depot)):
                    continue

                to_label = self.extended_label(from_label, to_cus)
                if not to_label:
                    from_label.unreachable_cs.add(to_cus)
                    continue
                
                if to_label.is_dominated():
                    continue

                to_label.filter_dominated()
                to_cus.labels.append(to_label)
                heappush(to_be_extended, to_label)
        
        min_label = min(depot.labels, key=lambda x: x.cost)
        return (min_label.path, min_label.cost)