from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

windowWidth, windowHeight = 500, 500  # Window size

def initOpenGL():
    glClearColor(1.0, 1.0, 1.0, 1.0)  # Set clear color to white
    gluOrtho2D(0, windowWidth, 0, windowHeight)  # Set coordinate system

def display():
    glClear(GL_COLOR_BUFFER_BIT)  # Clear the window
    glColor3f(0.0, 0.0, 0.0)  # Set text color to black

    # Position "Hello" in the center of the window
    x = windowWidth / 2
    y = windowHeight / 2
    glRasterPos2f(x, y)

    for char in "Hello":
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, ord(char))

    glFlush()  # Ensure all OpenGL commands are executed

def main():
    glutInit(sys.argv)
    glutInitDisplayMode(GLUT_RGB)
    glutInitWindowSize(windowWidth, windowHeight)
    glutInitWindowPosition(100, 100)
    glutCreateWindow(b'OpenGL "Hello" Window')

    initOpenGL()

    glutDisplayFunc(display)  # Register the display callback function

    glutMainLoop()

if __name__ == "__main__":
    main()
