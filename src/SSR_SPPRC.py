from ESPPRC import Label, ESPPRC

class SSR_Label(Label):
    """State space relaxed version of ESP Label. The relaxation is accomplished
       by not using the unreachable customer set as a resource, replacing it
       with the number of visited customers. By doing so the dominated labels
       grow in number and less paths are explored reducing the execution time,
       but it is possible to have cycles in the path found."""

    def __init__(self, *args):
        super().__init__(*args)
        self.n_visited = 0
    
    def dominates(self, label):
        return (super().dominates(label) and self.n_visited <= label.n_visited)

class SSR_SPPRC(ESPPRC):
    """State space relaxation SPPRC algorithm. See SSR label for details."""

    def __init__(self, *args):
        super().__init__(*args)
        self.label_cls = SSR_Label

    def extended_label(self, from_label, to_cus):
        label = super().extended_label(from_label, to_cus)
        if not label:
            return
            
        n_visited = from_label.n_visited + 1
        if n_visited > self.n_customers:
            return
        
        label.n_visited = n_visited
        return label