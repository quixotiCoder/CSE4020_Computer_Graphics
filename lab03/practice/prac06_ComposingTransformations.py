import glfw  # glfw 라이브러리를 가져옵니다. 이 라이브러리는 창 생성 및 이벤트 처리를 담당합니다.
from OpenGL.GL import *  # OpenGL 렌더링 함수를 가져옵니다.
from OpenGL.GLU import *  # OpenGL Utility Library 함수를 가져옵니다. (추가 기능을 제공합니다)
import numpy as np  # 수치 계산을 위한 NumPy를 가져옵니다.

gCamAng = 0.  # 전역 변수로 카메라 각도를 저장합니다.


def drawTriangleTransformedBy(M):
    """
    주어진 변환 행렬로 삼각형을 그리는 함수
    :param M: 변환 행렬
    """
    glBegin(GL_TRIANGLES)
    glVertex3fv((M @ np.array([.0, .5, 0., 1.]))[:-1])  # M이 왼쪽에 놓인다
    glVertex3fv((M @ np.array([.0, .0, 0., 1.]))[:-1])
    glVertex3fv((M @ np.array([.5, .5, 0., 1.]))[:-1])
    glEnd()


def drawTriangle():
    """
    변환 없이 삼각형을 그리는 함수
    """
    glBegin(GL_TRIANGLES)
    glVertex3fv(np.array([.0, .5, 0.]))
    glVertex3fv(np.array([.0, .0, 0.]))
    glVertex3fv(np.array([.5, .0, 0.]))
    glEnd()


def render(camAng):
    # 화면을 지우고 색상 및 깊이 버퍼를 초기화합니다.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # 깊이 테스트를 사용하여 올바른 깊이 지각을 보장합니다.
    glEnable(GL_DEPTH_TEST)

    # 현재 행렬을 항등 행렬로 초기화합니다.
    glLoadIdentity()

    # 직교 투영을 사용하여 2D 뷰를 설정합니다.
    glOrtho(-1, 1, -1, 1, -1, 1)

    # gluLookAt 함수를 사용하여 카메라의 위치와 방향을 설정합니다.
    gluLookAt(.1 * np.sin(camAng), .1, .1 * np.cos(camAng), 0, 0, 0, 0, 1, 0)

    # 좌표축을 그립니다.
    glBegin(GL_LINES)
    glColor3ub(255, 0, 0)  # 빨간색으로 설정합니다. (X축)
    glVertex3fv(np.array([0., 0., 0.]))
    glVertex3fv(np.array([1., 0., 0.]))
    glColor3ub(0, 255, 0)  # 초록색으로 설정합니다. (Y축)
    glVertex3fv(np.array([0., 0., 0.]))
    glVertex3fv(np.array([0., 1., 0.]))
    glColor3ub(0, 0, 255)  # 파란색으로 설정합니다. (Z축)
    glVertex3fv(np.array([0., 0., 0]))
    glVertex3fv(np.array([0., 0., 1.]))
    glEnd()

    ##############################
    # edit here

    # rotate 30 deg about x axis
    th = np.radians(30)
    R = np.identity(4)
    R[:3, :3] = [[1., 0., 0.],
                 [0., np.cos(th), -np.sin(th)],
                 [0., np.sin(th), np.cos(th)]]

    # translate by (.4, 0., .2)
    T = np.identity(4)
    T[:3, 3] = [.4, 0., .2]

    glColor3ub(255, 255, 255)

    '''# 1)
    glTranslatef(.4, .0, 0) # 먼저  (0.4, 0, 0) 만큼 이동
    glRotatef(60, 0, 0, 1) # z축을 중심으로 60도 회전 '''

    # 2) now swap the order
    glRotatef(60, 0, 0, 1) # 먼저 z축을 중심으로 60도 회전
    glTranslatef(.4, .0, 0) # (0.4, 0, 0) 만큼 이동

    drawTriangle()


def key_callback(window, key, scancode, action, mods):
    global gCamAng
    # 키가 눌리거나 반복될 때마다 카메라를 회전합니다.
    if action == glfw.PRESS or action == glfw.REPEAT:
        if key == glfw.KEY_1:  # 1 키를 누르면 시계 반대 방향으로 10도 회전합니다.
            gCamAng += np.radians(-10)
        elif key == glfw.KEY_3:  # 3 키를 누르면 시계 방향으로 10도 회전합니다.
            gCamAng += np.radians(10)


def main():
    if not glfw.init():  # glfw를 초기화합니다.
        return

    # 창을 생성합니다. (너비: 640, 높이: 640, 제목: 'OpenGL Trans. Functions')
    window = glfw.create_window(640, 640, 'OpenGL Trans. Functions', None, None)

    if not window:  # 창 생성에 실패하면 glfw를 종료합니다.
        glfw.terminate()
        return

    glfw.make_context_current(window)  # 현재 컨텍스트를 지정된 창으로 설정합니다.
    glfw.set_key_callback(window, key_callback)  # 키 콜백 함수를 설정합니다.

    while not glfw.window_should_close(window):  # 창이 닫힐 때까지 반복합니다.
        glfw.poll_events()  # 이벤트를 처리합니다.
        render(gCamAng)  # 렌더링 함수를 호출하여 화면을 그립니다.
        glfw.swap_buffers(window)  # 버퍼를 교체하여 그린 화면을 표시합니다.

    glfw.terminate()  # glfw를 종료합니다.


if __name__ == "__main__":
    main()
