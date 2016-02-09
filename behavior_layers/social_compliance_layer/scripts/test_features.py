
from __future__ import division

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Ellipse
import matplotlib.cm as cm
import matplotlib as mpl


def edist(v1, v2):
    return np.sqrt((v1[0] - v2[0])**2 + (v1[1] - v2[1])**2)


def distance_to_segment(x, xs, xe):
    xa = xs[0]
    ya = xs[1]
    xb = xe[0]
    yb = xe[1]
    xp = x[0]
    yp = x[1]

    # x-coordinates
    A = xb-xa
    B = yb-ya
    C = yp*B+xp*A
    a = 2*((B*B)+(A*A))
    b = -4*A*C+(2*yp+ya+yb)*A*B-(2*xp+xa+xb)*(B*B)
    c = 2*(C*C)-(2*yp+ya+yb)*C*B+(yp*(ya+yb)+xp*(xa+xb))*(B*B)
    if b*b < 4*a*c:
        return None, False
    x1 = (-b + np.sqrt((b*b)-4*a*c))/(2*a)
    x2 = (-b - np.sqrt((b*b)-4*a*c))/(2*a)

    # y-coordinates
    A = yb-ya
    B = xb-xa
    C = xp*B+yp*A
    a = 2*((B*B)+(A*A))
    b = -4*A*C+(2*xp+xa+xb)*A*B-(2*yp+ya+yb)*(B*B)
    c = 2*(C*C)-(2*xp+xa+xb)*C*B+(xp*(xa+xb)+yp*(ya+yb))*(B*B)
    if b*b < 4*a*c:
        return None, False
    y1 = (-b + np.sqrt((b*b)-4*a*c))/(2*a)
    y2 = (-b - np.sqrt((b*b)-4*a*c))/(2*a)

    # Put point candidates together
    xfm1 = np.array([x1, y1])
    xfm2 = np.array([x2, y2])
    xfm3 = np.array([x1, y2])
    xfm4 = np.array([x2, y1])

    dvec = list()
    dvec.append(edist(xfm1, x))
    dvec.append(edist(xfm2, x))
    dvec.append(edist(xfm3, x))
    dvec.append(edist(xfm4, x))

    dmax = -1.0
    imax = -1
    for i in xrange(4):
        if dvec[i] > dmax:
            dmax = dvec[i]
            imax = i

    xf = xfm1
    if imax == 0:
        xf = xfm1
    elif imax == 1:
        xf = xfm2
    elif imax == 2:
        xf = xfm3
    elif imax == 3:
        xf = xfm4

    xs_xf = np.array([xs[0]-xf[0], xs[1]-xf[1]])
    xe_xf = np.array([xe[0]-xf[0], xe[1]-xf[1]])
    dotp = (xs_xf[0] * xe_xf[0]) + (xs_xf[1] * xe_xf[1])

    inside = False
    if dotp <= 0.0:
        inside = True

    return dmax, inside


def angle_between(v1, v2):
    heading1 = np.arctan2(v1[1], v1[0])
    heading2 = np.arctan2(v2[1], v2[0])
    return normalize(heading1 - heading2)


def normalize(theta, start=0):
    if theta < np.inf:
        while theta >= start + 2 * np.pi:
            theta -= 2 * np.pi
        while theta < start:
            theta += 2 * np.pi
        return theta
    else:
        return np.inf


def _gaussianx(x, mu=0.0, sigma=0.2):
    fg = (1.0 / (sigma * np.sqrt(2*np.pi))) *\
        np.exp(-(x - mu)*(x - mu) / (2.0 * sigma * sigma))
    return fg


#############################################################################

def goal_deviation(source, target, goal, discount, duration):
    """ Goal deviation feature

    Penalizes heading away from the foal region by checking the
    heading of the chosen action. Discounted by the duration it
    takes to perform the action
    """
    v1 = np.array([target[0]-source[0], target[1]-source[1]], dtype=np.float64)
    v2 = np.array([goal[0]-source[0], goal[1]-source[1]], dtype=np.float64)
    goal_dev = angle_between(v1, v2) * discount ** duration
    return goal_dev


def social_intrusion(trajectory, persons, discount=0.99, itype='gaussian'):
    assert isinstance(trajectory, np.ndarray), 'np ndarray expected'
    sc = np.zeros(trajectory.shape[0])
    for i, p in enumerate(trajectory):
        if itype == 'flat':
            min_dist = 3.6
            for hp in persons:
                ed = edist(hp, p)
                if ed < min_dist:
                    min_dist = ed
            sc[i] = min_dist
            # continue
        else:
            for hp in persons:
                ed = edist(hp[0:2], p)
                if ed <= 4.0:
                    sc[i] += 3.6*_gaussianx(ed, sigma=0.24) * discount ** i

    return np.sum(sc)


def social_relation(trajectory, persons, relations,
                    discount=0.99, itype='uniform'):
    edc = np.zeros(trajectory.shape[0])
    for k, act in enumerate(trajectory):
        if itype == 'flat':
            min_dist = 3.6
            for [i, j] in relations:
                relation = ((persons[i-1][0], persons[i-1][1]),
                            (persons[j-1][0], persons[j-1][1]))

                d, inside = distance_to_segment(act, relation[0], relation[1])
                if inside:
                    if d < min_dist:
                        min_dist = d

            edc[k] = min_dist
            # continue
        else:
            for [i, j] in relations:
                relation = ((persons[i-1][0], persons[i-1][1]),
                            (persons[j-1][0], persons[j-1][1]))

                d, inside = distance_to_segment(act, relation[0], relation[1])
                if inside:
                    if d <= 1.2:
                        edc[k] += _gaussianx(d, mu=0.0, sigma=0.25)\
                                * discount**i

    return np.sum(edc)


def relative_heading_feature(robot, agent, goal, discount=0.99):
    pass


#############################################################################
# Make some people and lines

persons = [[2, 5, 2, 0],
           [5, 3.5, -0.5, 1.5],
           [6, 5, -1.5, 0],
           [5, 7.2, -0.5, -1.5],
           ]
relations = [
             [1, 2],
             # [1, 3],
             [1, 4],
             # [2, 4],
             [2, 3],
             # [3, 4]
             ]


STEP = 0.1


#############################################################################

def make_costmap():
    costmap = list()
    for x in np.arange(0, 10, STEP):
        for y in np.arange(0, 10, STEP):
            traj = np.array([[x, y]])
            sd = social_relation(traj, persons, relations, itype='flat')
            sc = social_intrusion(traj, persons, itype='flat')

            costmap.append([x, y, -1*(sd+sc)])
    costmap = np.array(costmap)
    return costmap


def main():
    figure = plt.figure(figsize=(10, 10))
    ax = plt.axes([0, 0, 1, 1])
    figure.add_axes(ax)

    for p in persons:
        phead = np.degrees(np.arctan2(p[3], p[2]))
        ax.add_artist(Ellipse((p[0], p[1]), width=0.3, height=0.8, angle=phead,
                      color='r', fill=False, lw=1.5, aa=True))
        ax.add_artist(Circle((p[0], p[1]), radius=0.2, color='w',
                      ec='r', lw=2.5, aa=True))
        ax.arrow(p[0], p[1], p[2]/5., p[3]/5., fc='r', ec='r', lw=1.5,
                 head_width=0.14, head_length=0.1)

    for [i, j] in relations:
        x1, x2 = persons[i-1][0], persons[i-1][1]
        y1, y2 = persons[j-1][0], persons[j-1][1]
        ax.plot((x1, y1), (x2, y2), ls='--', lw=2.0, c='r', alpha=0.7)

    plt.axis('equal')
    ax.set_xlim([0, 10])
    ax.set_ylim([0, 10])

    # ----------------- costmap -------------------------
    C = make_costmap()

    fig = plt.figure(figsize=(10, 10))
    # ax = fig.gca(projection='3d')
    ax = fig.gca()

    print(C[:, 2].shape)
    X = np.arange(0, 1, STEP)
    Y = np.arange(0, 1, STEP)
    X, Y = np.meshgrid(X, Y)

    a, b = 100, 100
    # a, b = C.shape
    xx, yy = np.meshgrid(np.arange(a), np.arange(b))
    z = np.zeros(shape=(a, b))
    cc = C[:, 2].reshape(a, b)
    for i in xx:
        for j in yy:
            z[i, j] = cc[i, j]
    z = np.nan_to_num(z)
    surf = ax.imshow(z.T, origin='lower', interpolation='bicubic',
                     cmap=plt.cm.jet)
    # ax.plot_surface(xx, yy, z, cmap=cm.jet, vmin=min(C[:, 2]),
    # vmax=max(C[:, 2]))

    ax.set_xticks([])
    ax.set_yticks([])
    plt.grid(False)
    fig.colorbar(surf, shrink=1, orientation='vertical')

    plt.show()


if __name__ == '__main__':
    main()
