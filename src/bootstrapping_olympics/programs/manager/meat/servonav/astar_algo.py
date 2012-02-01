from . import np, contract
from collections import namedtuple


NodeInfo = namedtuple('NodeInfo', 'parent cost_to_come min_cost_to_go')


def astar(start, target, node2children, heuristics):
    """ 
        start, target: nodes
        node2children: given a node, returns a list of tuples (children, cost)
        Returns a tuple (path, cost).     
        
        heuristics(node, target)
    """
    cost_to_come = {}

    nodes = {}
    nodes[target] = NodeInfo(None, np.inf, np.inf)
    nodes[start] = NodeInfo(None, 0, heuristics(start, target))
    open_nodes = set([start])

    while open_nodes:
        parent = argmin(open_nodes, lambda x: nodes[x].min_cost_to_go)
        open_nodes.remove(parent)

        assert nodes[parent].cost_to_come != np.inf

        for child, action_cost in node2children(parent):
            cost_to_come = nodes[parent].cost_to_come + action_cost
            if not np.isfinite(cost_to_come): continue

            min_cost_to_go = cost_to_come + heuristics(child, target)

            if ((not child in nodes) or
                (cost_to_come < nodes[child].cost_to_come and
                 min_cost_to_go < nodes[target].cost_to_come)):

                nodes[child] = NodeInfo(parent, cost_to_come, min_cost_to_go)
                open_nodes.add(child)

    if nodes[target].parent is None:
        return None, None
    else:
        path = [target]
        while nodes[path[0]].parent is not None:
            path.insert(0, nodes[path[0]].parent)

        return path, nodes[target].cost_to_come

def argmin(seq, key):
    return min([(x, key(x)) for x in seq], key=lambda c: c[1])[0]



@contract(node='tuple(int,int)', shape='tuple((int,>0),(int,>0))')
def node2children_grid(node, shape, cell_free, cost):
    ''' 
        cell_free((a,b)) should return True if the cell is free. 
        cost((a,b), (c,d)) returns the cost of the transition
    '''
    nodes = []
    def consider(a, b):
        if not ((0 <= a < shape[0]) and(0 <= b < shape[1])):
            return
        if cell_free((a, b)):
            nodes.append(((a, b), cost(node, (a, b))))

    i, j = node

    diff = [ [-1, -1], [-1, 0], [-1, +1],
             [0, -1], [0, +1],
             [+1, -1], [+1, 0], [+1, +1]]

    for di, dj in diff:
        consider(i + di, j + dj)
    return nodes
