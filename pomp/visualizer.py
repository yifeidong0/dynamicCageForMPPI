from __future__ import print_function,division
from builtins import range
from six import iteritems

from .klampt.glprogram import GLProgram
from .klampt import vectorops,so2,gldraw
from .spaces import metric
from .planners import allplanners
from .planners.kinodynamicplanner import EST
from .planners.optimization import iLQR
from .spaces.objectives import *
from OpenGL.GL import *
import os,errno
import time
import random
from OpenGL.GLUT import *

def mkdir_p(path):
    """Quiet path making"""
    try:
        os.makedirs(path)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise

class PlanVisualizationProgram(GLProgram):
    """Attributes:
        problem (PlanningProblem): the overall planning problem
        planner (multiple...): the planner selected for testing
        plannerFilePrefix (str): where to save testing data
    
    Internal attributes:
        path (Trajectory): if a solution is found, the best trajectory
        G (pair (V,E)): the roadmap for the planner.
        painted, save_movie, movie_frame: used for drawing / saving movies.
    """
    def __init__(self,problem,planner,plannerFilePrefix):
        GLProgram.__init__(self)
        self.problem = problem
        self.planner = planner
        self.plannerFilePrefix = plannerFilePrefix
        self.path = None
        self.G = None
        self.painted = False
        self.save_movie = False
        self.movie_frame = 0
        self.prev_path_x = None
        self.display_new_path = False

    def mousefunc(self,button,state,x,y):
        if state == 0:
            if hasattr(self.planner,'nextSampleList'):
                self.planner.nextSampleList.append(self.problem.visualizer.toState(float(x)/self.width,float(y)/self.height))
                print(self.planner.nextSampleList)
            self.refresh()

    def idlefunc(self):
        if self.save_movie:
            if self.painted:
                try:
                    mkdir_p(self.plannerFilePrefix)
                    self.save_screen("%s/image%04d.ppm"%(self.plannerFilePrefix,self.movie_frame))
                    self.movie_frame += 1
                except Exception as e:
                    ex_type, ex, tb = sys.exc_info()
                    traceback.print_tb(tb)
                    self.save_movie = False
                    raise e

                self.planner.planMore(100)
                self.path = self.planner.getPath()
                self.G = self.planner.getRoadmap()
                self.refresh()
                self.painted = False
               
    def keyboardfunc(self,key,x,y):
        key = key.decode("utf-8") 
        print("Key",key,"pressed at",x,y)
        if key==' ':
            print("Planning 1...")
            self.planner.planMore(1)
            self.path = self.planner.getPath()
            self.G = self.planner.getRoadmap()
            self.planner.stats.pretty_print()
            self.refresh()
        elif key=='p':
            print("Planning 1000...")
            self.planner.planMore(1000)
            self.path = self.planner.getPath()
            self.G = self.planner.getRoadmap()
            self.planner.stats.pretty_print()
            self.refresh()
        elif key=='o':
            if self.path is not None:
                costWeightInit = 0.1
                xinit,uinit = self.path
            else:
                costWeightInit = 0.1
                xinit,uinit,cost = self.planner.getBestPath(self.problem.objective + SetDistanceObjectiveFunction(self.problem.goal) + StepCountObjectiveFunction()*(-0.1))
            print("Optimizing 10...")
            objective = self.problem.objective
            if isinstance(objective,PathLengthObjectiveFunction):
                objective = EnergyObjectiveFunction()
            if self.problem.controlSpace is None:
                from .spaces.controlspace import ControlSpaceAdaptor
                optimizer = iLQR(ControlSpaceAdaptor(self.problem.configurationSpace),objective,self.problem.goal,'log',costWeightInit)
            else:
                optimizer = iLQR(self.problem.controlSpace,objective,self.problem.goal,'log',costWeightInit)
            converged,reason = optimizer.run(xinit,uinit,20)
            print("Converged?",converged,"Termination reason",reason)
            print("Endpoints",optimizer.xref[0],optimizer.xref[-1])
            self.path = optimizer.xref,optimizer.uref
            self.refresh()
        elif key=='r':
            print("Resetting planner")
            self.planner.reset()
            self.path = None
            self.G = self.planner.getRoadmap()
            self.refresh()
        elif key=='m':
            self.save_movie = not self.save_movie
            if self.save_movie:
                self.refresh()
        elif key=='t':
            maxTime = 30
            # testPlanner(self.planner,10,maxTime,self.plannerFilePrefix+'.csv')
       
    def display(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0,1,1,0,-1,1);
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glEnable(GL_POINT_SMOOTH)
        glDisable(GL_DEPTH_TEST)

        glDisable(GL_LIGHTING)
        self.problem.visualizer.drawObstaclesGL()

        if hasattr(self.planner,'nextSampleList'):
            for p in self.planner.nextSampleList:
                self.problem.visualizer.drawRobotGL(p)

        self.draw_graph()

        #Debug extension cache
        est = None
        if isinstance(self.planner,EST): est = self.planner
        elif hasattr(self.planner,'est'): est = self.planner.est
        if est and hasattr(est,'extensionCache'):
            glLineWidth(3.0)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
            sumw = sum(est.extensionCache[0])
            for (w,(n,u,e)) in zip(*est.extensionCache):
                glColor4f(1,0,1,(0.1+0.9*w/sumw))
                self.problem.visualizer.drawInterpolatorGL(e)
                #self.problem.visualizer.drawRobotGL(e.end())
            glDisable(GL_BLEND)
            glLineWidth(1.0)

        # Debug density
        """
        if est:
            for n in est.nodes:
                if est.density(n.x) <= 1.0:
                    self.problem.visualizer.drawRobotGL(n.x)
        """

        # Draw static/animated path found by the planner
        self.draw_path_animation()

    def draw_graph(self):
        if self.G:            #draw graph
            V,E = self.G
            glLineWidth(0.5)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
            glColor4f(0,0,0,0.5)
            glPointSize(3.0)
            self.problem.visualizer.drawVerticesGL(V)
            glColor4f(0.5,0.5,0.5,0.5)
            for (i,j,e) in E:
                interpolator = self.problem.space.interpolator(V[i],e)
                self.problem.visualizer.drawInterpolatorGL(interpolator)
            glDisable(GL_BLEND)

    def draw_path_animation(self):
        if hasattr(self.problem.controlSpace, "is_moving_obstacle"):
            if self.path is not None:
                if not are_nested_lists_equal(self.prev_path_x, self.path[0]):
                    self.display_new_path = True
                    self.prev_path_x = self.path[0]
            
            if self.display_new_path:
                self.problem.visualizer.drawRobotGL(self.problem.start)
                self.problem.visualizer.drawGoalGL(self.problem.goal)

                if self.path and len(self.path[0]) > 1:
                    # print("x", self.path[0])
                    # print("u", self.path[1])
                    for k in range(len(self.path[0]) - 1):
                        if k >= len(self.path[0])-1:
                            break
                        x1, u = self.path[0][k], self.path[1][k]
                        x2 = self.path[0][k+1]

                        # Clear the previous obstacle by drawing a background color (e.g., white)
                        glColor3f(1, 1, 1)
                        self.problem.visualizer.drawObstaclesGL(x1[4:6])

                        # Draw the new obstacle at q2
                        glColor3f(0.2,0.2,0.2)
                        self.problem.visualizer.drawObstaclesGL(x2[4:6])

                        # Draw the graph again
                        self.draw_graph()

                        for m in range(k+1):
                            x1, u = self.path[0][m], self.path[1][m]
                            x2 = self.path[0][k+1]
                            # Interpolate and draw all previous line segments
                            glColor3f(0, 0.75, 0)
                            glLineWidth(7.0)
                            interpolator = self.problem.space.interpolator(x1, u)
                            self.problem.visualizer.drawInterpolatorGL(interpolator)
                            # Draw the node x2
                            glLineWidth(1)
                            self.problem.visualizer.drawRobotGL(x2)

                        # Process events to update the display
                        glutSwapBuffers()
                        glutMainLoopEvent()

                        # Pause and save screen
                        if self.display_new_path:
                            time.sleep(0.1*u[0]) # <0.05*u[0] might cause frame chaos in the generated movies
                        if self.save_movie:
                            self.save_screen("%s/image%04d.ppm"%(self.plannerFilePrefix,self.movie_frame))
                            self.movie_frame += 1

            else: # draw static path without animation if no path update
                self.draw_path_static()

            self.display_new_path = False

        if hasattr(self.problem.controlSpace, "is_cage_planner"):
            if self.path is not None:
                if not are_nested_lists_equal(self.prev_path_x, self.path[0]):
                    self.display_new_path = True
                    self.prev_path_x = self.path[0]
            
            if self.display_new_path:
                # print("x", self.path[0])
                # print("u", self.path[1])
                self.problem.visualizer.drawRobotGL(self.problem.start)
                self.problem.visualizer.drawGoalGL(self.problem.goal)

                if self.path and len(self.path[0]) > 1:
                    for k in range(len(self.path[0]) - 1):
                        if k >= len(self.path[0])-1:
                            break
                        x1, u = self.path[0][k], self.path[1][k]
                        x2 = self.path[0][k+1]

                        # Clear the previous obstacle by drawing a background color (e.g., white)
                        glColor3f(1, 1, 1)
                        self.problem.visualizer.drawGripperGL(x1[4:7], self.problem.controlSpace.half_extents_gripper)

                        # Draw the new obstacle at x2
                        glColor3f(0.2,0.2,0.2)
                        self.problem.visualizer.drawGripperGL(x2[4:7], self.problem.controlSpace.half_extents_gripper)

                        # Draw the graph again
                        self.draw_graph()

                        for m in range(k+1):
                            x1, u = self.path[0][m], self.path[1][m]
                            x2 = self.path[0][k+1]
                            # Interpolate and draw all previous line segments
                            glColor3f(0, 0.75, 0)
                            glLineWidth(7.0)
                            interpolator = self.problem.space.interpolator(x1, u)
                            self.problem.visualizer.drawInterpolatorGL(interpolator)
                            # Draw the node x2
                            glLineWidth(1)
                            self.problem.visualizer.drawRobotGL(x2)

                        # Process events to update the display
                        glutSwapBuffers()
                        glutMainLoopEvent()

                        # Pause and save screen
                        if self.display_new_path:
                            time.sleep(0.1*u[0]) # <0.05*u[0] might cause frame chaos in the generated movies
                        if self.save_movie:
                            self.save_screen("%s/image%04d.ppm"%(self.plannerFilePrefix,self.movie_frame))
                            self.movie_frame += 1

            else: # draw static path without animation if no path update
                self.draw_path_static()

            self.display_new_path = False

        else: # cases other than cageMO
            self.draw_path_static()

    def draw_path_static(self):
        if self.path:
            # Plot path edges
            glColor3f(0,0.75,0)
            glLineWidth(7.0)
            for q,u in zip(self.path[0][:-1],self.path[1]):
                interpolator = self.problem.space.interpolator(q,u)
                self.problem.visualizer.drawInterpolatorGL(interpolator)

            # Plot path nodes
            glLineWidth(1)
            for q in self.path[0]:
                self.problem.visualizer.drawRobotGL(q)
            
        # Draw start and goal region
        self.problem.visualizer.drawRobotGL(self.problem.start)
        self.problem.visualizer.drawGoalGL(self.problem.goal)

    def displayfunc(self):
        GLProgram.displayfunc(self)
        self.painted = True

def are_nested_lists_equal(list1, list2):
    # Check if the lengths of the lists are equal
    if list1 is None or list2 is None:
        return False
    if len(list1) != len(list2):
        return False

    # Iterate through the elements of the lists
    for i in range(len(list1)):
        if isinstance(list1[i], list) and isinstance(list2[i], list):
            if not are_nested_lists_equal(list1[i], list2[i]):
                return False
        else:
            if list1[i] != list2[i]:
                return False

    return True

def runVisualizer(problem,**plannerParams):   
    if 'type' in plannerParams:
        plannerType = plannerParams['type']
        del plannerParams['type']
    else:
        plannerType = 'r-rrt'

    planner = problem.planner(plannerType,**plannerParams)
    mkdir_p("data")
    program = PlanVisualizationProgram(problem,planner,os.path.join("data",allplanners.filename[plannerType]))
    program.width = program.height = 640
    program.run()


