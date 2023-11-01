from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

def draw_box():
    glBegin(GL_QUADS)
    # Front face
    glColor3f(1.0, 0.0, 0.0)  # Red
    glVertex3f(-1.0, -1.0, 1.0)
    glVertex3f(1.0, -1.0, 1.0)
    glVertex3f(1.0, 1.0, 1.0)
    glVertex3f(-1.0, 1.0, 1.0)

    # Back face
    glColor3f(0.0, 0.0, 1.0)  # Blue
    glVertex3f(-1.0, -1.0, -1.0)
    glVertex3f(1.0, -1.0, -1.0)
    glVertex3f(1.0, 1.0, -1.0)
    glVertex3f(-1.0, 1.0, -1.0)

    # Left face
    glColor3f(0.0, 1.0, 0.0)  # Green
    glVertex3f(-1.0, -1.0, -1.0)
    glVertex3f(-1.0, 1.0, -1.0)
    glVertex3f(-1.0, 1.0, 1.0)
    glVertex3f(-1.0, -1.0, 1.0)

    # Right face
    glColor3f(1.0, 1.0, 0.0)  # Yellow
    glVertex3f(1.0, -1.0, -1.0)
    glVertex3f(1.0, 1.0, -1.0)
    glVertex3f(1.0, 1.0, 1.0)
    glVertex3f(1.0, -1.0, 1.0)

    # Top face
    glColor3f(0.0, 1.0, 1.0)  # Cyan
    glVertex3f(-1.0, 1.0, -1.0)
    glVertex3f(1.0, 1.0, -1.0)
    glVertex3f(1.0, 1.0, 1.0)
    glVertex3f(-1.0, 1.0, 1.0)

    # Bottom face
    glColor3f(1.0, 0.0, 1.0)  # Magenta
    glVertex3f(-1.0, -1.0, -1.0)
    glVertex3f(1.0, -1.0, -1.0)
    glVertex3f(1.0, -1.0, 1.0)
    glVertex3f(-1.0, -1.0, 1.0)
    glEnd()

# def display():
#     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
#     glLoadIdentity()
#     glTranslatef(0.0, 0.0, -6.0)
#     glRotatef(45, 1, 1, 0)  # Rotate the box
#     draw_box()
#     glutSwapBuffers()

def reshape(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(width) / float(height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)

# Global variables for the line
line_start = (-1.0, -1.0, -1.0)
line_end = (1.0, 1.0, 1.0)
line_color = (0.0, 1.0, 0.0)  # Green color

# def draw_box():
#     # ... (same as before)

def draw_line():
    glColor3fv(line_color)
    glBegin(GL_LINES)
    glVertex3fv(line_start)
    glVertex3fv(line_end)
    glEnd()

def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0.0, 0.0, -6.0)
    glRotatef(45, 1, 1, 0)  # Rotate the box
    draw_box()
    draw_line()  # Draw the line
    glutSwapBuffers()

def idlefunc():
    global line_start, line_end
    # Update the line's position (for example, move it along the z-axis)
    line_start = (line_start[0], line_start[1], line_start[2] + 0.01)
    line_end = (line_end[0], line_end[1], line_end[2] + 0.01)
    glutPostRedisplay()  # Trigger a display refresh

def main():
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutInitWindowSize(800, 600)
    glutCreateWindow("OpenGL Box and Line Example")
    glEnable(GL_DEPTH_TEST)
    glutDisplayFunc(display)
    glutIdleFunc(idlefunc)  # Register the idle function
    glutReshapeFunc(reshape)
    glutMainLoop()

if __name__ == "__main__":
    main()