import glfw
from OpenGL.GL import *
import numpy as np

def render(T): # Draw a triangle using render() function below (DO NOT modify it!). 의 코드로 render
    glClear(GL_COLOR_BUFFER_BIT)
    glLoadIdentity()
    # draw coordinate
    glBegin(GL_LINES)
    glColor3ub(255, 0, 0)
    glVertex2fv(np.array([0.,0.]))
    glVertex2fv(np.array([1.,0.]))
    glColor3ub(0, 255, 0)
    glVertex2fv(np.array([0.,0.]))
    glVertex2fv(np.array([0.,1.]))
    glEnd()
    # draw triangle
    glBegin(GL_TRIANGLES)
    glColor3ub(255, 255, 255)
    glVertex2fv( (T @ np.array([.0,.5,1.]))[:-1] )
    glVertex2fv( (T @ np.array([.0,.0,1.]))[:-1] )
    glVertex2fv( (T @ np.array([.5,.0,1.]))[:-1] )
    glEnd()

def main():
    if not glfw.init():
        return
    
    window = glfw.create_window(480,480,"2022072560-2-2", None,None) # Set the window title to [studentID]-[assignment#]-[prob#].(e.g. 2017123456-2-2) and the window size to (480,480). 조건 충족
    
    if not window:
        glfw.terminate()
        return
    
    glfw.make_context_current(window)

    # set the number of screen refresh to wait before calling glfw.swap_buffer().
    # if your monitor refresh rate is 60Hz, the while loop is repeated every 1/60 sec
    glfw.swap_interval(1)

    glfw.swap_interval(1)

    while not glfw.window_should_close(window):
        glfw.poll_events()

        # get the current time, in seconds
        t = glfw.get_time()

        th = t
        

        T = np.array([[1., 0., .5], # 이동 변환
                      [0., 1., 0.],
                      [0., 0., 1.]])

        R = np.array([[np.cos(th), -np.sin(th), 0.], # 회전 변환
                      [np.sin(th), np.cos(th), 0.],
                      [0., 0., 1.]])



        render(R @ T) # 선 이동 후 회전

        glfw.swap_buffers(window)

    glfw.terminate()

if __name__ == "__main__":
    main()
