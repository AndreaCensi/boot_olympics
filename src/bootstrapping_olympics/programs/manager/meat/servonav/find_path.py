from . import astar, node2children_grid, contract, np
from geometry import (SE2_from_SE3, translation_from_SE2,
    SE2_from_translation_angle, SE2_from_xytheta, SE3_from_SE2)
import itertools


@contract(resolution='float,>0')
def get_grid(robot, world, vehicle, resolution):
    from vehicles.simulation.collision import collides_with
    # TODO: check Vehicle
    bounds = world.bounds
    bx = bounds[0]
    by = bounds[1]
    wx = float(bx[1] - bx[0])
    wy = float(by[1] - by[0])
    nx = int(np.ceil(wx / resolution))
    ny = int(np.ceil(wy / resolution))

    cx = wx / nx
    cy = wy / ny

    primitives = world.get_primitives()
    vehicle.set_world_primitives(primitives)

    grid = np.zeros((nx, ny), int)
    grid.fill(-1)

    locations = []

    for i, j in itertools.product(range(nx), range(ny)):
        x = bx[0] + (i + 0.5) * cx
        y = by[0] + (j + 0.5) * cy
        theta = 0
        pose = SE3_from_SE2(SE2_from_xytheta([x, y, theta]))
        loc = dict(cell=(i, j), pose=pose)
        grid[i, j] = len(locations)
        locations.append(loc)

    def collision_at(pose):
        center = translation_from_SE2(SE2_from_SE3(pose))
        collision = collides_with(primitives, center=center,
                                  radius=vehicle.radius * 2.5)
        #print('center %s: %s' % (center, collision))
        return collision.collided

    def cell_free(node):
        loc = locations[grid[node]]
        if not 'collision' in loc:
            loc['collision'] = collision_at(loc['pose'])
            #print('Checking %s: %s' % (node, loc['collision']))

        return not loc['collision']

    def cost(node1, node2):
        p1 = np.array(node1)
        p2 = np.array(node2)
        dist = np.linalg.norm(p1 - p2)
        return dist

    def node2children(node):
        return node2children_grid(node, shape=grid.shape,
                                        cell_free=cell_free,
                                        cost=cost)

    def heuristics(node, target):
        return cost(node, target)

    def get_start_cell():
        for a in range(len(locations)):
            cell = locations[a]['cell']
            if cell_free(cell):
                return cell
        else:
            raise Exception("No free space at all")

    def get_target_cells():
        """ Enumerate end cells rom the bottom """
        found = False
        for a in range(len(locations)):
            k = len(locations) - 1 - a
            cell = locations[k]['cell']
            if cell_free(cell):
                found = True
                yield cell
        if not found:
            raise Exception("No free space at all")

    start = get_start_cell()

    # Find a long path 
    for target in get_target_cells():
        path, _ = astar(start, target, node2children, heuristics)

        if path is not None:
            break
    else:
        raise Exception('Could not find any path.')

    locations = [locations[grid[c]] for c in path]

    # adjust poses angle
    for i in range(len(locations) - 1):
#        if i == 0:
#            loc1 = locations[i - 1]
#        else:
        loc1 = locations[i]
        loc2 = locations[i + 1]

#        if i == len(locations) - 1:
#            loc1 = locations[-2]
#            loc2 = locations[-1]
#        else:

        t1 = translation_from_SE2(SE2_from_SE3(loc1['pose']))
        t2 = translation_from_SE2(SE2_from_SE3(loc2['pose']))
        d = t2 - t1
        theta = np.arctan2(d[1], d[0])
        diff = SE3_from_SE2(SE2_from_translation_angle(t=[0, 0], theta=theta))
        loc1['pose'] = np.dot(loc1['pose'], diff)

    # compute observations
    print('Found path of length %s' % len(locations))
    for loc in locations:
        pose = loc['pose']
        robot.vehicle.set_pose(pose)
        loc['observations'] = mean_observations(robot, n=5)

    return locations


def mean_observations(robot, n):
    """ Averages the robot's observations at the given place. """
    # XXX: only works for 1D?
    obss = []
    for _ in range(n): # XXX: fixed threshold
        obss.append(robot.get_observations().observations)
    return np.mean(obss, axis=0)

