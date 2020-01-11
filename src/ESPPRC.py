import numpy as np
from collections import deque
from functools import lru_cache

class Label:
    """A label describes a path from the depot to a customer and the resources
       used in this path.
       
       Labels are associated with a customer and are used to identify each
       feasible state in which that customer can be reached.
       
       A dominance relation between labels is needed (see 'dominates' method for
       more information) so only 'best' labels can be kept on a node resulting
       in fewer paths to be explored.
       
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
        self.unreachable_cs = set()
        self.prev = prev
        self.dominated = False

    def __repr__(self):
        return f"{(self.customer, self.cost, self.load, self.time)}"

    @property
    @lru_cache(maxsize=1)
    def path(self):
        """Returns the path described by this label."""

        label = self
        path = []
        while label.prev:
            path.append(label.customer)
            label = label.prev
        path.append(label.customer)
        return list(reversed(path))
	
    def dominates(self, label):
        """A label is called 'dominant' when compared to another label, if
           it uses less resources than the other label, meaning that all of
           its resources must be less than or equal to the other's, but there
           must be at least one resource that is different.

           Since two perfectly equals labels describe the same path, it is
           possible to throw away one of them by modifying the above definition
           and letting l1 dominate l2 even if they have the same resources.
           
           Arguments:
               label: the label to compare with
           Returns:
               True if this label dominates 'label', False otherwise
        """

        return (self.cost <= label.cost and self.load <= label.load
                and self.time <= label.time)

    def is_dominated(self):
        """Returns True if this label is dominated by any of the labels
           associated with the same customer, False otherwise."""

        for label in self.customer.labels:
            if label.dominates(self):
                return True
        return False

    def filter_dominated(self):
        """Removes labels dominated by this label on its customer."""

        labels = []
        for label in self.customer.labels:
            if self.dominates(label):
                # label can be already in the 'to_be_extended' queue
                # so we need to signal that this label is no more extendable
                label.dominated = True
            else:
                labels.append(label)
        self.customer.labels = labels

class ESP_Label(Label):
    """Extension of base Label used to describe elementary paths only.
       To do that the unreachable customers set contains also the visited
       customers and it is used as a resource, meaning that the dominance
       relation is also extended."""

    def __init__(self, *args):
        super().__init__(*args)
        self.unreachable_cs.add(self.customer)
    
    def dominates(self, label):
        # Note that having the unreachable customers set as a resource means
        # that a label uses less of this resource if it possesses a subset of
        # the other's unreachable customers set.
        return (super().dominates(label)
                and self.unreachable_cs.issubset(label.unreachable_cs))

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
        self.customers = set(customers)
        self.costs = costs
        self.times = times
        self.depot = customers[0]
        self.duals = np.zeros(len(customers))
        self.n_customers = len(customers)
        self.label_cls = ESP_Label

    def solve(self):
        for customer in self.customers:
            customer.labels = []
        to_be_extended = deque([self.depot_label()])
        while to_be_extended:
            from_label = to_be_extended.popleft()
            # if a label becomes dominated after being pushed in the queue,
            # label.dominated becomes true and it can be skipped
            if from_label.dominated:
                continue
            
            to_labels = self.feasible_labels_from(from_label)
            for to_label in to_labels:
                to_cus = to_label.customer
                if to_cus is not self.depot:
                    to_label.unreachable_cs.update(from_label.unreachable_cs)
                    if to_label.is_dominated():
                        continue
                    to_label.filter_dominated()
                    to_be_extended.append(to_label)
                to_cus.labels.append(to_label)

        return sorted(self.depot.labels, key=lambda x: x.cost)

    def depot_label(self):
        """Returns the algorithm starting label. It has no resources and its
           path can return to the depot."""

        label = self.label_cls(self.depot, 0, 0, 0)
        label.unreachable_cs.clear()
        return label

    def feasible_labels_from(self, from_label):
        """Arguments:
               from_label: the label that is going to be extended.
           Returns:
               A list of feasible labels that extends 'from_label'.
           Note: 'from_label' unreachable set is updated in the process.
        """

        to_labels = []
        for to_cus in (self.customers - from_label.unreachable_cs
                                      - {from_label.customer}):
            to_label = self.extended_label(from_label, to_cus)
            if not to_label:
                from_label.unreachable_cs.add(to_cus)
            else:
                to_labels.append(to_label)
        return to_labels

    def extended_label(self, from_label, to_cus):
        """Returns a new label that extends 'from_label' and goes to 'to_cus'.
           
           Arguments:
               from_label: the label to start from
               to_cus: the customer to reach
           Returns:
               A new label with updated resources or None if some resource
               exceeds its limits.
        """

        load = from_label.load + to_cus.demand
        if load > self.capacity:
            return
        
        from_cus = from_label.customer
        time = max(from_label.time + from_cus.service_time
                                   + self.times[from_cus, to_cus],
                   to_cus.time_window[0])
        if time > to_cus.time_window[1]:
            return

        cost = (from_label.cost + self.costs[from_cus, to_cus]
                                - self.duals[from_cus])
        # unreachable customers update is delayed since from_label needs to
        # visit every customer before knowing its own set
        return self.label_cls(to_cus, cost, load, time, from_label)