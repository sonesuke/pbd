

# forall vertices i
#     initialize xi = xi_0, vi = vi_0, wi = 1 / mi
# endfor
#
# loop
#     forall vertices i do vi <- vi + dt * wi * fext(xi)
#     damp_veloctieis(vi,....,vN)
#     forall vertices i do pi <- xi + dt * vi
#     forall vertices i do generate_collision_constraints(xi -> pi)
#
#     loop solver_iterartion times
#         project_constraints(C1,...,CM+M_coll, p1,...,pN)
#     endloop
#
#     forall vertices i
#         vi <- (pi - xi) / dt
#         xi <- pi
#     endfor
#     velocity_update(v1, ..., vN)
# endloop

import numpy as np

class Particle:

    def __init__(self, x, v, m):
        self.x = np.array(x)
        self.v = np.array(v)
        self.w = 1 / m
        self.p = np.array(x)

    def __repr__(self):
        return "Particle: " + "x: " + str(self.x) + ", v: " + str(self.v) + ", w: " + str(self.w)


class FixedParticle:

    def __init__(self, x):
        self.x = np.array(x)
        self.v = np.array([0, 0, 0])
        self.w = 0
        self.p = np.array(x)

    def __repr__(self):
        return "Fixed   : " + "x: " + str(self.x) + ", v: " + str(self.v) + ", w: " + str(self.w)


def dump_velocity(p):
    return p.v * 0.9

def distance_constraints(p1, p2, d):
    p1p2 = np.dot(p1.p - p2.p, p1.p - p2.p)
    p1p2 = np.sqrt(p1p2)
    t = (p1p2 - d) * (p1.p - p2.p) / p1p2 / (p1.w + p2.w)
    p1.p -= p1.w * t
    p2.p += p2.w * t


def step_pbd(particles, dt, fext, iteration):
    for p in particles:
        p.v += dt * p.w * fext
        p.v = dump_velocity(p)

    for p in particles:
        p.p = p.x + dt * p.v

    for i in range(iteration):
        distance_constraints(particles[0], particles[3], 5)
        distance_constraints(particles[1], particles[4], 5)
        distance_constraints(particles[2], particles[5], 5)
        distance_constraints(particles[3], particles[6], 5)
        distance_constraints(particles[4], particles[7], 5)
        distance_constraints(particles[5], particles[8], 5)
        distance_constraints(particles[6], particles[9], 0)
        distance_constraints(particles[7], particles[9], 0)
        distance_constraints(particles[8], particles[9], 0)

    for p in particles:
        p.v = (p.p - p.x) / dt
        p.x = p.p

np.set_printoptions(precision=4)

d = 1
particles = [
    FixedParticle([d * np.cos(0), d * np.sin(0), 0]),
    FixedParticle([d * np.cos(2 * np.pi / 3), d * np.sin(2 * np.pi / 3), 0]),
    FixedParticle([d * np.cos(4 * np.pi / 3), d * np.sin(4 * np.pi / 3), 0]),
    Particle([0, 0, 0], [0, 0, 0], 1),
    Particle([0, 0, 0], [0, 0, 0], 1),
    Particle([0, 0, 0], [0, 0, 0], 1),
    Particle([0, 0, 0], [0, 0, 0], 1),
    Particle([0, 0, 0], [0, 0, 0], 1),
    Particle([0, 0, 0], [0, 0, 0], 1),
    FixedParticle([0, 0, 0]),
    ]

ts = np.linspace(0, 2 * np.pi, 100)
xs = np.sin(ts)
ys = np.cos(ts)

for i in range(100):
    particles[-1].x = np.array([xs[i], ys[i], 3])
    step_pbd(particles, 0.1, 0, 5)
    txt = ""
    for p in particles:
        txt += "%f, %f, %f, " % (p.x[0], p.x[1], p.x[2])
    print txt

