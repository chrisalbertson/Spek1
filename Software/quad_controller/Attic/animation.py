"""Package to display a real-time amimated image of the robot"""

import logging
log = logging.getLogger(__name__)

import matplotlib.pyplot as plt
import matplotlib.animation as animation


def setupView(limit):
    ax = plt.axes(projection="3d")
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_xlabel("X")
    ax.set_ylabel("Z")
    ax.set_zlabel("Y")
    return ax

def get_neural_stance_Lp() -> np.array:
    """Function to return a reasonable neutral stance on flat ground"""

    # Units for these are Meters.
    vertical =      0.180
    stance_width =  0.120
    stance_length = 0.260

    stance_halfwidth = stance_width / 2.0
    stance_halflength = stance_length / 2.0

    # Distances of each foot from body center
    #                   fore/aft         height       left/right
    Lp = np.array([[ stance_halflength, -vertical,  stance_halfwidth, 1],   # LF
                   [ stance_halflength, -vertical, -stance_halfwidth, 1],   # RF
                   [-stance_halflength, -vertical,  stance_halfwidth, 1],   # LR
                   [-stance_halflength, -vertical, -stance_halfwidth, 1]])  # RR
    return Lp

def drawLegPoints(self, p):
    plt.plot([x[0] for x in p], [x[2] for x in p], [x[1] for x in p], 'k-', lw=3)
    plt.plot([p[0][0]], [p[0][2]], [p[0][1]], 'bo', lw=2)
    plt.plot([p[4][0]], [p[4][2]], [p[4][1]], 'ro', lw=2)


def drawLegPair(self, Tl, Tr, Ll, Lr):
    Ix = np.array([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    self.drawLegPoints([Tl.dot(x) for x in self.calcLegPoints(self.legIK(np.linalg.inv(Tl).dot(Ll)))])
    self.drawLegPoints(
        [Tr.dot(Ix.dot(x)) for x in self.calcLegPoints(self.legIK(Ix.dot(np.linalg.inv(Tr).dot(Lr))))])


def drawRobot(self, Lp, angles: (float, float, float), center: (float, float, float)):
    (omega, phi, psi) = angles
    (xm, ym, zm) = center

    FP = [0, 0, 0, 1]
    (Tlf, Trf, Tlb, Trb) = self.bodyIK(omega, phi, psi, xm, ym, zm)
    CP = [x.dot(FP) for x in [Tlf, Trf, Tlb, Trb]]

    CPs = [CP[x] for x in [0, 1, 3, 2, 0]]
    plt.plot([x[0] for x in CPs], [x[2] for x in CPs], [x[1] for x in CPs], 'bo-', lw=2)

    self.drawLegPair(Tlf, Trf, Lp[0], Lp[1])
    self.drawLegPair(Tlb, Trb, Lp[2], Lp[3])


def animate():
    angles = (0.0, 0.0, 0.0)
    center = (0.0, 0.0, 0.0)
    Lp = get_neural_stance_Lp()
    drawRobot(Lp, angles, center)


if __name__ == "__main__":
    logging.basicConfig(filename='quad_controller.log',
                        filemode='w',
                        level=logging.DEBUG, )
    ax = setupView(1.0)
    ani = animation.FuncAnimation(ax,
                                  animate,
                                  fargs=(),
                                  interval=100,
                                  blit=True)
