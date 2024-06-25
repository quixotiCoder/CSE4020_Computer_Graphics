import glfw
from OpenGL.GL import *
import numpy as np

input_key = GL_LINE_LOOP # When the program starts를 위한 초기 case 처리

def render():
    glClear(GL_COLOR_BUFFER_BIT) 
    glLoadIdentity()

    angle = np.linspace(0, 2*np.pi, 12, endpoint = False) # 정12각형을 위해 각도 : 360도(2pi)를 12개로 나눔. np.linspace 함수 사용.  'endpoint = False'조건 -> 0 = 2pi가 같아지는 점 포함 X
    vertices = np.array([np.cos(angle), np.sin(angle)]).T # vertices의 각 점을 cos / sin 함수로 나타내어 사분면 반시계 방향 조건 만족 & 각각 [x, y] 좌표에 대응시키기 위해 '.T' 로 transpose하여 좌표로 나타냄


    glBegin(input_key) # keyboard input에 따라 Primitive Types 조정
    
    for vertex in vertices: # vertices라는 array 속 각 원소들(= x, y 좌표쌍)을 하나씩
        glVertex2fv(vertex) # 2D 좌표계 상 vertex로 찍기
        
    glEnd()

def key_callback(window, key, scancode, action, mods): # keyboard input마다 달라지는 events 출력을 위해 key_callback 선언 
    global input_key # Hint: Use a global variable to store the primitive type 조건 충
    if action == glfw.PRESS:
        
        if key == glfw.KEY_1:
            input_key = GL_POINTS

        elif key == glfw.KEY_2:
            input_key = GL_LINES

        elif key == glfw.KEY_3:
            input_key = GL_LINE_STRIP

        elif key == glfw.KEY_4:
            input_key = GL_LINE_LOOP

        elif key == glfw.KEY_5:
            input_key = GL_TRIANGLES

        elif key == glfw.KEY_6:
            input_key = GL_TRIANGLE_STRIP

        elif key == glfw.KEY_7:
            input_key = GL_TRIANGLE_FAN

        elif key == glfw.KEY_8:
            input_key = GL_QUADS

        elif key == glfw.KEY_9:
            input_key = GL_QUAD_STRIP

        elif key == glfw.KEY_0:
            input_key = GL_POLYGON


def main():
    # Initialize the library
    if not glfw.init():
        return
    # Create a windowed mode window and its OpenGL context
    window = glfw.create_window(480, 480, "2022072560-2-1", None, None) # Set the window title to [studentID]-[assignment#]-[prob#] (e.g. 2017123456-2-1) and the window size to (480,480). 조건 충족
    if not window:
        glfw.terminate()
        return

    glfw.set_key_callback(window, key_callback)
    
    # Make the window's context current 
    glfw.make_context_current(window)
    

    # Loop until the user closes the window
    while not glfw.window_should_close(window):
        # Poll events
        glfw.poll_events()

        # Render here, e.g. using pyOpenGL
        render()

        # Swap front and back buffers
        glfw.swap_buffers(window)


    glfw.terminate()

if __name__ == "__main__":
    main()
