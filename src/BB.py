from collections import namedtuple
from heapq import *
import math

class BBNode:
    def __lt__(self, bbnode):
        return self.obj < bbnode.obj
    
    def __iter__(self):
        yield self.obj
        yield self.solution
    
def BB(root_bbnode):
    min_obj = math.inf
    min_bbnode = None
    priority_queue = []
    solve_and_push(priority_queue, root_bbnode)
    while priority_queue:
        bbnode = heappop(priority_queue)
        if bbnode.is_integer():
            if bbnode.obj < min_obj:
                min_bbnode = bbnode
                min_obj = bbnode.obj
            continue
        if bbnode.obj > min_obj:
            continue
        left_bbnode, right_bbnode = bbnode.split()
        solve_and_push(priority_queue, left_bbnode)
        solve_and_push(priority_queue, right_bbnode)
    return min_bbnode

def solve_and_push(priority_queue, bbnode):
    bbnode.solve()
    if not bbnode.is_infeasible():
        heappush(priority_queue, bbnode)