import math

class EdgeChecker:
    def feasible(self,interpolator):
        """ interpolator: a subclass of Interpolator.
        Returns true if all points along the interpolator are feasible.
        """
        raise NotImplementedError()

class EpsilonEdgeChecker(EdgeChecker):
    """An edge checker that uses a fixed resolution for collision checking.
    """
    def __init__(self,space,resolution):
        """Arguments:
            - space: a subclass of ConfigurationSpace
            - resolution: an edge checking resolution
        """
        self.space = space # it means C-space, MultiConfigurationSpace
        self.resolution = resolution
    def feasible(self,interpolator,movingObstacles=False):
        l = interpolator.length()
        k = int(math.ceil(l / self.resolution))
        # print("interpolator.start()", interpolator.start())
        # print("interpolator.end()", interpolator.end())
        # obs_pos = obs_pos_init + x[4:]
        if not self.space.feasible(interpolator.start(), movingObstacles) or not self.space.feasible(interpolator.end(), movingObstacles):
            return False
        for i in range(k):
            u = float(i+1)/float(k+2)
            x = interpolator.eval(u)
            # print("interpolator.x", x)
            if not self.space.feasible(x, movingObstacles):
                return False
        return True
