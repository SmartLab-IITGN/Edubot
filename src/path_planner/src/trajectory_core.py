from matplotlib import bezier
import numpy as np

class Waypoint :

    def __init__(
        self,
        x = 0,
        y = 0,
        theta = 0,
        v = 0.001,
        w = 0,
        t = np.inf
    ) :
        self.x = float(x)
        self.y = float(y)
        self.theta = np.clip(theta, -np.pi, np.pi)

        self.v = float(v)
        self.w = float(w)

        self.t = float(t)

        pass

class PolyCurve :

    def __init__(self) -> None:
        
        self.__x_poly = np.zeros(4)
        self.__y_poly = np.zeros(4)

        self.__x_dot_poly = np.zeros(3)
        self.__y_dot_poly = np.zeros(3)

        pass

    def generatePoly(self, waypoint_1, waypoint_2, c = 1.0) :

        self.__x_poly[3] = waypoint_1.x
        self.__y_poly[3] = waypoint_1.y

        self.__x_poly[2] = waypoint_1.v * np.cos(waypoint_1.theta) / c
        self.__y_poly[2] = waypoint_1.v * np.sin(waypoint_1.theta) / c

        self.__x_poly[1] = 3.0 * (waypoint_2.x - waypoint_1.x) - (2.0 * waypoint_1.v * np.cos(waypoint_1.theta) + waypoint_2.v * np.cos(waypoint_2.theta)) / c
        self.__y_poly[1] = 3.0 * (waypoint_2.y - waypoint_1.y) - (2.0 * waypoint_1.v * np.sin(waypoint_1.theta) + waypoint_2.v * np.sin(waypoint_2.theta)) / c

        self.__x_poly[0] = 2.0 * (waypoint_1.x - waypoint_2.x) + (waypoint_1.v * np.cos(waypoint_1.theta) + waypoint_2.v * np.cos(waypoint_2.theta))
        self.__y_poly[0] = 2.0 * (waypoint_1.y - waypoint_2.y) + (waypoint_1.v * np.sin(waypoint_1.theta) + waypoint_2.v * np.sin(waypoint_2.theta))

        self.__x_dot_poly[:] = np.polyder(self.__x_poly)
        self.__y_dot_poly[:] = np.polyder(self.__y_poly)

        pass

    def getVal(self, s) :

        s = np.clip(s, 0.0, 1.0)
        
        return np.array([
            np.polyval(self.__x_poly, s),
            np.polyval(self.__y_poly, s)
        ])

    def getDerivative(self, s) :

        s = np.clip(s, 0.0, 1.0)

        return np.array([
            np.polyval(self.__x_dot_poly, s),
            np.polyval(self.__y_dot_poly, s)
        ])

class TrajectoryPlanner :

    def __init__(self) -> None:
        
        self.__waypoint_list = []

        self.__max_time = np.inf

        self.x_poly = np.zeros(4)
        self.y_poly = np.zeros(4)

        pass

    def addWaypoint(self, waypoint) :

        self.__waypoint_list.append(waypoint)

        pass

    def getRefPos(self, t) :

        if t < self.__max_time :

            pass

        pass

if __name__ == '__main__' :

    import matplotlib.pyplot as plt

    p1 = Waypoint(theta=np.pi/2, v=1)

    p2 = Waypoint(x=2, y=1)

    my_curve = PolyCurve()
    my_curve.generatePoly(p1, p2)

    s = np.linspace(0, 1)

    print(my_curve.getVal(1))

    points = my_curve.getVal(s)
    plt.plot(points[0], points[1])

    plt.show()    

