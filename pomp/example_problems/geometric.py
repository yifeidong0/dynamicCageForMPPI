from OpenGL.GL import *
from ..klampt import gldraw
from ..spaces.objectives import *
from ..spaces.configurationspace import *
from ..spaces.edgechecker import *
from ..spaces.metric import *
from ..spaces.configurationspace import NeighborhoodSubset,SingletonSubset
from ..planners.problem import PlanningProblem

class Circle:
    def __init__(self,x=0,y=0,radius=1):
        self.center = (x,y)
        self.radius = radius
        
    def contains(self,point):
        return (vectorops.distance(point,self.center) <= self.radius)

    def signedDistance(self,point):
        return (vectorops.distance(point,self.center) - self.radius)
    
    def signedDistance_gradient(self,point):
        d = vectorops.sub(point,self.center)
        return vectorops.div(d,vectorops.norm(d))

    def drawGL(self,res=0.01):
        numdivs = int(math.ceil(self.radius*math.pi*2/res))
        glBegin(GL_TRIANGLE_FAN)
        glVertex2f(*self.center)
        for i in range(numdivs+1):
            u = float(i)/float(numdivs)*math.pi*2
            glVertex2f(self.center[0]+self.radius*math.cos(u),self.center[1]+self.radius*math.sin(u))
        glEnd()


class Box(BoxSet):
    def __init__(self,x1=0,y1=0,x2=0,y2=0):
        BoxSet.__init__(self,[min(x1,x2),min(y1,y2)],[max(x1,x2),max(y1,y2)])
        
    def drawGL(self):
        glBegin(GL_QUADS)
        glVertex2f(*self.bmin)
        glVertex2f(self.bmax[0],self.bmin[1])
        glVertex2f(*self.bmax)
        glVertex2f(self.bmin[0],self.bmax[1])
        glEnd()


class AxisNotAlignedBox:
    def __init__(self, gripperPose, halfExtent):
        self.center_x = gripperPose[0]
        self.center_y = gripperPose[1]
        self.theta = gripperPose[2]
        self.half_length = halfExtent[0] # x
        self.half_height = halfExtent[1] # z

    def drawGL(self):
        # Calculate the corner points of the rotated box
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)

        p1 = (self.center_x - cos_theta * self.half_length + sin_theta * self.half_height,
              self.center_y + sin_theta * self.half_length + cos_theta * self.half_height)
        
        p2 = (self.center_x + cos_theta * self.half_length + sin_theta * self.half_height,
              self.center_y - sin_theta * self.half_length + cos_theta * self.half_height)

        p3 = (self.center_x + cos_theta * self.half_length - sin_theta * self.half_height,
              self.center_y - sin_theta * self.half_length - cos_theta * self.half_height)

        p4 = (self.center_x - cos_theta * self.half_length - sin_theta * self.half_height,
              self.center_y + sin_theta * self.half_length - cos_theta * self.half_height)

        glBegin(GL_QUADS)
        glVertex2f(*p1)
        glVertex2f(*p2)
        glVertex2f(*p3)
        glVertex2f(*p4)
        glEnd()

    def contains(self, point):
        """point: list[2]"""
        # Translate the point to the box's coordinate system
        translated_x = point[0] - self.center_x
        translated_y = point[1] - self.center_y

        # Rotate the point in the opposite direction of the box
        cos_theta = math.cos(-self.theta)
        sin_theta = math.sin(-self.theta)

        rotated_x = translated_x * cos_theta - translated_y * sin_theta
        rotated_y = translated_x * sin_theta + translated_y * cos_theta

        # Check if the rotated point is within the axis-aligned box
        return (abs(rotated_x) <= self.half_length) and (abs(rotated_y) <= self.half_height)

class Geometric2DCSpace(BoxConfigurationSpace):
    def __init__(self):
        BoxConfigurationSpace.__init__(self,[0,0],[1,1])
        self.obstacles = []
        self.obstacleParams = None

    def addObstacleParam(self,params):
        self.obstacleParams = params

    def addObstacle(self,obs):
        self.obstacles.append(obs) # initial obstacle params

    def feasible(self,x,obstaclePos=None):
        """Collision detection.
            input - x: list[2], position of object. 
        """
        if not BoxConfigurationSpace.feasible(self,x): return False
        if self.obstacleParams is None: # static obstacles
            for o in self.obstacles:
                if o.contains(x): return False
            return True
        elif len(self.obstacleParams[0])==4: # moving obstacles with horizontal gripper
            for i in range(len(self.obstacles)):
                topLeftX = self.obstacleParams[i][0] + obstaclePos[0]
                topLeftY = self.obstacleParams[i][1] + obstaclePos[1]
                o = Box(topLeftX,
                        topLeftY,
                        topLeftX+self.obstacleParams[i][2],
                        topLeftY+self.obstacleParams[i][3]
                        )
                if o.contains(x): return False
            return True
        else: # len(self.obstacleParams[0])==5, tilting gripper
            gripperPose = obstaclePos # xc, yc, theta
            halfExtent = self.obstacleParams[0][3:]
            o = AxisNotAlignedBox(gripperPose, halfExtent)
            if o.contains(x): return False
            return True

    def toScreen(self,q):
        return (q[0]-self.box.bmin[0])/(self.box.bmax[0]-self.box.bmin[0]),(q[1]-self.box.bmin[1])/(self.box.bmax[1]-self.box.bmin[1])

    def toState(self,x,y):
        return (self.box.bmin[0]+x*(self.box.bmax[0]-self.box.bmin[0]),
                self.box.bmin[1]+y*(self.box.bmax[1]-self.box.bmin[1]))

    def beginDraw(self):
        if self.box.bmin != [0,0] or self.box.bmin != [1,1]:
            glPushMatrix()
            glScalef(1.0/(self.box.bmax[0]-self.box.bmin[0]),1.0/(self.box.bmax[1]-self.box.bmin[1]),1)
            glTranslatef(-self.box.bmin[0],-self.box.bmin[1],0)

    def endDraw(self):
        if self.box.bmin != [0,0] or self.box.bmin != [1,1]:
            glPopMatrix()

    def drawObstaclesGL(self, obstaclePos=None):
        self.beginDraw()
        if obstaclePos is None:
            glColor3f(0.2,0.2,0.2)
            for o in self.obstacles:
                o.drawGL()
        else: # moving obstacles
            for i in range(len(self.obstacles)):
                topLeftX = self.obstacleParams[i][0] + obstaclePos[0]
                topLeftY = self.obstacleParams[i][1] + obstaclePos[1]
                o = Box(topLeftX,
                        topLeftY,
                        topLeftX+self.obstacleParams[i][2],
                        topLeftY+self.obstacleParams[i][3]
                        )
                o.drawGL()
        self.endDraw()

    def drawGripperGL(self, gripperPose, halfExtent):
        """For cagePlanner and cageEL"""
        self.beginDraw()
        gripper = AxisNotAlignedBox(gripperPose, halfExtent)
        gripper.drawGL()
        self.endDraw()

    def drawVerticesGL(self,qs):
        self.beginDraw()
        glBegin(GL_POINTS)
        for q in qs:
            glVertex2f(q[0],q[1])
        glEnd()
        self.endDraw()

    def drawRobotGL(self,q):
        glColor3f(0,0,1)
        glPointSize(7.0)
        self.drawVerticesGL([q])

    def drawGoalGL(self,goal):
        self.beginDraw()
        if isinstance(goal,NeighborhoodSubset):
            q = goal.c
            glColor3f(1,0,0)
            gldraw.circle(q,goal.r,filled=False)
            glPointSize(7.0)
            glBegin(GL_POINTS)
            glVertex2f(q[0],q[1])
            glEnd()
        elif isinstance(goal,SingletonSubset):
            q = goal.x
            glColor3f(1,0,0)
            glPointSize(7.0)
            glBegin(GL_POINTS)
            glVertex2f(q[0],q[1])
            glEnd()
        else:
            glColor3f(1,0,0)
            glPointSize(7.0)
            glBegin(GL_POINTS)
            for i in range(50):
                q = goal.sample()
                glVertex2f(q[0],q[1])
            glEnd()
        self.endDraw()

    def drawInterpolatorGL(self,interpolator):
        self.beginDraw()
        if isinstance(interpolator,LinearInterpolator):
            #straight line paths
            glBegin(GL_LINES)
            glVertex2f(interpolator.a[0],interpolator.a[1])
            glVertex2f(interpolator.b[0],interpolator.b[1])
            glEnd()
        elif isinstance(interpolator,PiecewiseLinearInterpolator):
            glBegin(GL_LINE_STRIP)
            for x in interpolator.path:
                glVertex2f(x[0],x[1])
            glEnd()
        else:
            glBegin(GL_LINE_STRIP)
            for s in range(10):
                u = float(s) / (9.0)
                x = interpolator.eval(u)
                glVertex2f(x[0],x[1])
            glEnd()
        self.endDraw()

    def clearance(self,x):
        res = BoxConfigurationSpace.clearance(self,x)
        for o in self.obstacles:
            res.append(o.signedDistance(x))
        return res
    
    def clearance_gradient(self,x):
        res = [BoxConfigurationSpace.clearance_gradient(self,x)]
        for o in self.obstacles:
            res.append(o.signedDistance_gradient(x))
        return np.vstack(res)


def circleTest():
    space = Geometric2DCSpace()
    space.addObstacle(Circle(0.5,0.4,0.39))
    start=[0.06,0.25]
    goal=[0.94,0.25]
    objective = PathLengthObjectiveFunction()
    goalRadius = 0.1
    return PlanningProblem(space,start,goal,
                           objective=objective,
                           visualizer=space,
                           costLowerBound = vectorops.distance,
                           goalRadius = goalRadius,
                           euclidean = True)


def rrtChallengeTest():
    w = 0.03
    eps = 0.01
    space = Geometric2DCSpace()
    space.box = BoxSet([0,0],[1,w*2+eps])
    space.addObstacle(Box(0,w*2+eps,1,1))
    space.addObstacle(Box(w,w,1,w+eps))
    start=[1-w*0.5,w+eps+w*0.5]
    goal=[1-w*0.5,w*0.5]
    goalRadius = w*0.5
    objective = PathLengthObjectiveFunction()
    return PlanningProblem(space,start,goal,
                           objective=objective,
                           visualizer=space,
                           costLowerBound = vectorops.distance,
                           goalRadius = goalRadius,
                           euclidean = True)

def kinkTest():
    space = Geometric2DCSpace()
    w = 0.02
    space.addObstacle(Box(0.3,0,0.5+w*0.5,0.2-w*0.5))
    space.addObstacle(Box(0.5+w*0.5,0,0.7,0.3-w*0.5))
    space.addObstacle(Box(0.3,0.2+w*0.5,0.5-w*0.5,0.7))
    space.addObstacle(Box(0.5-w*0.5,0.3+w*0.5,0.7,0.7))
    start=[0.06,0.25]
    goal=[0.94,0.25]
    goalRadius = 0.1
    objective = PathLengthObjectiveFunction()
    return PlanningProblem(space,start,goal,
                           objective=objective,
                           visualizer=space,
                           costLowerBound = vectorops.distance,
                           goalRadius = goalRadius,
                           euclidean = True)

def bugtrapTest():
    space = Geometric2DCSpace()
    w = 0.1
    """
    space.addObstacle(Box(0.55,0.25,0.6,0.75))
    space.addObstacle(Box(0.15,0.25,0.55,0.3))
    space.addObstacle(Box(0.15,0.7,0.55,0.75))
    space.addObstacle(Box(0.15,0.25,0.2,0.5-w*0.5))
    space.addObstacle(Box(0.15,0.5+w*0.5,0.2,0.75))
    start=[0.5,0.5]
    goal=[0.65,0.5]
    """
    start=[0.1,0.5]
    goal=[0.9,0.5]
    space.addObstacle(Box(0.3,0.3,0.7,0.7))
    goalRadius = 0.1
    objective = PathLengthObjectiveFunction()
    return PlanningProblem(space,start,goal,
                           objective=objective,
                           visualizer=space,
                           costLowerBound = vectorops.distance,
                           goalRadius = goalRadius,
                           euclidean = True)

