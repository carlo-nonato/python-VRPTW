from SSR_SPPRC import SSR_Label, SSR_SPPRC

def find_duplicates(items):
	seen, seen_twice = set(), set()
	for item in items:
		if item in seen:
			seen_twice.add(item)
		else:
			seen.add(item)
	return seen_twice

class DSSR_Label(SSR_Label):
    def __init__(self, *args):
        super().__init__(*args)
        self.critical_visited = set()
    
    def dominates(self, label):
        return (super().dominates(label)
                and self.critical_visited.issubset(label.critical_visited))

class DSSR_ESPPRC(SSR_SPPRC):
    def __init__(self, *args):
        super().__init__(*args)
        self.critical_cs = set()
    
    def solve(self):
        while True:
            path, cost = super().solve()
            duplicates = find_duplicates(path[:-1])
            if duplicates:
                self.critical_cs.update(duplicates)
            else:
                return path, cost
    
    def extended_label(self, from_label, to_cus):
        label = super().extended_label(from_label, to_cus)
        if not label:
            return

        if to_cus in self.critical_cs:
            if to_cus in label.critical_visited:
                return
            label.critical_visited.add(to_cus)
        return label