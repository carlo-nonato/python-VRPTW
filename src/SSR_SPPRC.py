from ESPPRC import Label, ESPPRC

class SSR_Label(Label):
    def __init__(self, *args):
        super().__init__(*args)
        self.n_visited = 0
    
    def dominates(self, label):
        return (super().dominates(label) and self.n_visited <= label.n_visited)

class SSR_SPPRC(ESPPRC):
    def __init__(self, *args):
        super().__init__(*args)
        self.label_cls = SSR_Label

    def extended_label(self, from_label, to_cus):
        label = super().extended_label(from_label, to_cus)
        if not label:
            return

        label.n_visited += 1*(to_cus is not self.depot)
        if label.n_visited <= len(self.customers):
            return label