import glfw
from OpenGL.GL import *
import numpy as np
from OpenGL.GLU import *


def render():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glOrtho(-2, 2, -2, 2, -1, 1)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    drawFrame()
    t = glfw.get_time()
##################
    # blue base transformation
    glPushMatrix() ## 1
    glTranslatef(np.sin(t), 0, 0) # t에 따라 x축 이동

    # blue base drawing
    glPushMatrix() ## 2
    glScalef(.2, .2, .2) # blue 박스 0.2배
    glColor3ub(0, 0, 255) # blue 색상 설정
    drawBox()
    glPopMatrix()  ## -1

    glPushMatrix()
    drawFrame()
    glPopMatrix()

#####################

    # red arm transformation
    glPushMatrix() ## 2
    glRotatef(t * (180 / np.pi), 0, 0, 1) # 회전
    glTranslatef(.5, 0, .01) # 중점좌표 이동 z축 .01만큼 blue보다 위

    # red arm drawing
    glPushMatrix() ## 3
    glScalef(.5, .1, .1)
    glColor3ub(255, 0, 0)
    drawBox()
    glPopMatrix()

    glPushMatrix()
    drawFrame()
    glPopMatrix()
#########################
    # green arm transformation
    glPushMatrix()
    glTranslatef(.5, 0, .01) # 1st, translate
    glRotatef(t * (180 / np.pi), 0, 0, 1) # 2nd, rotate


    # green box
    glPushMatrix()
    glScalef(.2, .2, .2)  # same size with blue box
    glColor3ub(0, 255, 0)
    drawBox()
    glPopMatrix()

    glPushMatrix()
    drawFrame()
    glPopMatrix()

######################
    glPopMatrix()
    glPopMatrix()
    glPopMatrix()

def drawBox():
    glBegin(GL_QUADS)
    glVertex3fv(np.array([1,1,0.]))
    glVertex3fv(np.array([-1,1,0.]))
    glVertex3fv(np.array([-1,-1,0.]))
    glVertex3fv(np.array([1,-1,0.]))
    glEnd()

def drawFrame():
    # draw coordinate: x in red, y in green, z in blue
    glBegin(GL_LINES)
    glColor3ub(255, 0, 0)
    glVertex3fv(np.array([0.,0.,0.]))
    glVertex3fv(np.array([1.,0.,0.]))
    glColor3ub(0, 255, 0)
    glVertex3fv(np.array([0.,0.,0.]))
    glVertex3fv(np.array([0.,1.,0.]))
    glColor3ub(0, 0, 255)
    glVertex3fv(np.array([0.,0.,0]))
    glVertex3fv(np.array([0.,0.,1.]))
    glEnd()

def main():
    if not glfw.init():
        return

    window = glfw.create_window(480,480,'2022072560-lab4-1', None,None)

    if not window:
        glfw.terminate()
        return

    glfw.make_context_current(window)
    glfw.swap_interval(1)

    while not glfw.window_should_close(window):
        glfw.poll_events()
        render()
        glfw.swap_buffers(window)

    glfw.terminate()

if __name__ == "__main__":
    main()