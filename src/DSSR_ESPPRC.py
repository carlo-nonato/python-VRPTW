from SSR_SPPRC import SSR_Label, SSR_SPPRC

def find_repeated(items):
    """Arguments:
           items: the items to be searched for repeated elements.
       Returns:
           A set of elements that are repeated in 'items'."""

    seen, seen_twice = set(), set()
    for item in items:
        if item in seen:
            seen_twice.add(item)
        else:
            seen.add(item)
    return seen_twice

class DSSR_Label(SSR_Label):
    """Decremental state space relaxed version of ESP Label. Same as SSR Label
       but with an added resource, the critical customers visited.
       Critical customers are customers that the algorithm previously find as
       repeated in the SSR SPPRC execution.
       The new resource doesn't allow to visit critical customers twice (but
       only them), so it prevents the same cycle to appear again."""

    def __init__(self, *args):
        super().__init__(*args)
        self.critical_visited = set()
    
    def dominates(self, label):
        return (super().dominates(label)
                and self.critical_visited.issubset(label.critical_visited))

class DSSR_ESPPRC(SSR_SPPRC):
    """Decremental state space relaxation ESPPRC algorithm. It starts by solving
       the SPPRC using DSSR labels, which are basically SSR labels, then if it
       finds a cycle (repeated customers in the resulting path) it keeps tracks
       of those customers (called critical customers) and restart the algorithm.
       The DSSR labels used (see the respective documentation) prevent critical
       customers to be visited twice so the process is repeated until an acyclic
       path is returned."""

    def __init__(self, *args):
        super().__init__(*args)
        self.label_cls = DSSR_Label
        self.critical_cs = set()
    
    def solve(self):
        while True:
            path, cost = super().solve()
            repeated = find_repeated(path[:-1])
            if repeated:
                self.critical_cs.update(repeated)
            else:
                return path, cost
    
    def extended_label(self, from_label, to_cus):
        label = super().extended_label(from_label, to_cus)
        if not label or to_cus in from_label.critical_visited:
           return

        if to_cus in self.critical_cs:
            label.critical_visited = from_label.critical_visited | {to_cus}
        return label