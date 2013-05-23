from . import astar, node2children_grid
from contracts import describe_type
from geometry import SE3, SE2, angle_from_SE2, SE2_from_SE3
import itertools

from contracts import contract
import numpy as np

@contract(resolution='float,>0')
def get_grid(robot, vsim, resolution, debug=False):
    # Note: use robot to get the observations, be cause it might 
    # include some nuisances that you don't get if you use vsim 
    # directly
    from vehicles import VehicleSimulation
    if not isinstance(vsim, VehicleSimulation):
        msg = ('I really require a VehicleSimulation; obtained %s.' % 
               describe_type(vsim))
        raise ValueError(msg)

    world = vsim.world
    vehicle = vsim.vehicle

    from geometry import (translation_from_SE2,
        SE2_from_translation_angle, SE2_from_xytheta, SE3_from_SE2)
    from vehicles.simulation.collision import collides_with

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
        # print('center %s: %s' % (center, collision))
        return collision.collided

    def cell_free(node):
        loc = locations[grid[node]]
        if not 'collision' in loc:
            loc['collision'] = collision_at(loc['pose'])
            # print('Checking %s: %s' % (node, loc['collision']))

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

    def get_start_cells():
        order = range(len(locations))[::5]
        for a in order:
            cell = locations[a]['cell']
            if cell_free(cell):
                yield cell
        else:
            raise Exception("No free space at all")

    def get_target_cells(start):
        """ Enumerate end cells rom the bottom """
        def goodness(cell):
            return cost(start, cell)

        order = np.argsort([-goodness(l['cell']) for l in locations])

        found = False
        for k in order[::5]:
            cell = locations[k]['cell']
            if cell_free(cell):
                found = True
                # print('TRying %s (%s)' % (str(cell), goodness(cell)))
                yield cell
        if not found:
            raise Exception("No free space at all")

    # Find a long path 
    def get_path():
        for start in get_start_cells():
            for target in get_target_cells(start):
                path, _ = astar(start, target, node2children, heuristics)
                if path:
                    return path
        else:
            raise Exception('Could not find any path.')

    path = get_path()

    locations = [locations[grid[c]] for c in path]

    # adjust poses angle
    for i in range(len(locations) - 1):
        loc1 = locations[i]
        loc2 = locations[i + 1]
        t1 = translation_from_SE2(SE2_from_SE3(loc1['pose']))
        t2 = translation_from_SE2(SE2_from_SE3(loc2['pose']))
        d = t2 - t1
        theta = np.arctan2(d[1], d[0])
        diff = SE3_from_SE2(SE2_from_translation_angle(t=[0, 0], theta=theta))
        loc1['pose'] = np.dot(loc1['pose'], diff)

    poses = [l['pose'] for l in locations]

    # project to SE2
    poses_se2 = map(SE2_from_SE3, poses)
    # smooth path
    poses_se2 = elastic(SE2, poses_se2, alpha=0.1, num_iterations=20)
    poses = map(SE3_from_SE2, poses_se2)

    # compute observations
    print('Found path of length %s' % len(poses))

    # poses = interpolate_sequence(poses, max_theta_diff_deg=30)
    # compute observations
    print('Upsampled at %s' % len(poses))

    locations = [dict(pose=pose) for pose in poses]
    for loc in locations:
        pose = loc['pose']
        vehicle.set_pose(pose)
        if not debug:
            loc['observations'] = mean_observations(robot, n=10)

    return locations


# def get_random_path(find_path):
#    ''' Find a path between two cells. 
#            find_path(start, target)
#    '''
#    for start in get_start_cells():
#        for target in get_target_cells(start):
#            path, _ = astar(start, target, node2children, heuristics)
#
#            if path is not None:
#                break
#    else:
#        raise Exception('Could not find any path.')


@contract(pose1='SE3', pose2='SE3', max_theta_diff_deg='>0')
def step_too_big(pose1, pose2, max_theta_diff_deg):
    diff = SE3.multiply(SE3.inverse(pose1), pose2)
    diff_theta = np.abs(angle_from_SE2(SE2_from_SE3(diff)))
    return diff_theta > np.deg2rad(max_theta_diff_deg)


@contract(pose1='SE3', pose2='SE3', returns='SE3')
def interpolate(pose1, pose2):
    return SE3.geodesic(pose1, pose2, 0.5)


@contract(poses='list[N,>=2](SE3)', returns='list[>=N](SE3)')
def interpolate_sequence(poses, max_theta_diff_deg):
    poses = list(poses)
    inter = []
    inter.append(poses.pop(0))
    while True:
        if not poses:
            break
        current_pose = inter[-1]
        next_pose = poses[0]
        if step_too_big(current_pose, next_pose, max_theta_diff_deg):
            interpolated = interpolate(current_pose, next_pose)
            inter.append(interpolated)
        else:
            inter.append(poses.pop(0))
    return inter


def mean_observations(robot, n):
    """ Averages the robot's observations at the given place. """
    # XXX: only works for 1D?
    obss = []
    for _ in range(n):  # XXX: fixed threshold
        obss.append(robot.get_observations().observations)

    mean = np.mean(obss, axis=0)

    return mean


@contract(poses='list[>=3](array[KxK])')
def elastic(manifold, poses, alpha=0.1, num_iterations=20):
    poses2 = [x.copy() for x in poses]

    for _ in range(num_iterations):
        for i, cur in enumerate(list(poses2)):
            if i == 0 or i == len(poses2) - 1:
                continue

            pA = poses2[i - 1]
            pB = poses2[i + 1]
            target = manifold.geodesic(pA, pB, 0.5)
            moved = manifold.geodesic(cur, target, alpha)
#            if i == 5:
#                print('Now: %s' % SE2.friendly(SE2_from_SE3(moved)))
#            moved[:2, 3] = cur[:2, 3] # don't move 
            poses2[i] = moved

    return poses2


