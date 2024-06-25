import glfw  # 창 생성 및 이벤트 처리를 위한 glfw 라이브러리를 가져옵니다.
from OpenGL.GL import *  # 렌더링을 위한 OpenGL 함수를 가져옵니다.
from OpenGL.GLU import *  # 추가 기능을 위한 OpenGL Utility Library 함수를 가져옵니다.
import numpy as np  # 수치 계산을 위한 NumPy를 가져옵니다.


def render(M, camAng):
    # 색상 및 깊이 버퍼를 지웁니다. (자세한 내용은 나중에 다룰 것입니다)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # 올바른 깊이 지각을 위해 깊이 테스트를 활성화합니다. (자세한 내용은 나중에 다룰 것입니다)
    glEnable(GL_DEPTH_TEST)

    # 변환을 초기화하기 위해 항등 행렬을 로드합니다. (자세한 내용은 나중에 다룰 것입니다)
    glLoadIdentity()

    # 지정된 범위 내에서 직교 투영을 설정합니다 (2D 뷰) (자세한 내용은 나중에 다룰 것입니다)
    glOrtho(-1, 1, -1, 1, -1, 1)

    # gluLookAt 함수를 사용하여 카메라 위치를 설정합니다. (자세한 내용은 나중에 다룰 것입니다)
    gluLookAt(.1 * np.sin(camAng), .1, .1 * np.cos(camAng), 0, 0, 0, 0, 1, 0)

    # 좌표축을 그립니다: x는 빨간색, y는 초록색, z는 파란색
    glBegin(GL_LINES)
    glColor3ub(255, 0, 0)  # 색상을 빨간색으로 설정합니다.
    glVertex3fv(np.array([0., 0., 0.]))  # x축의 시작점
    glVertex3fv(np.array([1., 0., 0.]))  # x축의 끝점
    glColor3ub(0, 255, 0)  # 색상을 초록색으로 설정합니다.
    glVertex3fv(np.array([0., 0., 0.]))  # y축의 시작점
    glVertex3fv(np.array([0., 1., 0.]))  # y축의 끝점
    glColor3ub(0, 0, 255)  # 색상을 파란색으로 설정합니다.
    glVertex3fv(np.array([0., 0., 0.]))  # z축의 시작점
    glVertex3fv(np.array([0., 0., 1.]))  # z축의 끝점
    glEnd()

    # 삼각형을 그립니다.
    glBegin(GL_TRIANGLES)
    glColor3ub(255, 255, 255)  # 색상을 흰색으로 설정합니다.
    glVertex3fv((M @ np.array([.0, .5, 0., 1.]))[:-1])  # 정점 1
    glVertex3fv((M @ np.array([.0, .0, 0., 1.]))[:-1])  # 정점 2
    glVertex3fv((M @ np.array([.5, .0, 0., 1.]))[:-1])  # 정점 3
    glEnd()


def main():
    # glfw 라이브러리를 초기화합니다.
    if not glfw.init():
        return

    # 지정된 창 크기와 제목으로 창을 생성합니다.
    window = glfw.create_window(640, 640, "3D Trans", None, None)

    # 창 생성이 성공했는지 확인합니다.
    if not window:
        glfw.terminate()
        return

    # 지정된 창의 컨텍스트를 현재로 만들어 렌더링합니다.
    glfw.make_context_current(window)
    # 창의 스왑 간격을 설정합니다 (버퍼 스왑을 제어합니다).
    glfw.swap_interval(1)

    while not glfw.window_should_close(window):
        # 이벤트(키보드, 마우스 등)를 처리합니다.
        glfw.poll_events()
        # 현재 시간을 가져옵니다.
        t = glfw.get_time()

        # x축 주위로 60도 회전합니다.
        th = np.radians(-60)
        R = np.identity(4)
        R[:3, :3] = [[1., 0., 0.], [0., np.cos(th), -np.sin(th)], [0., np.sin(th), np.cos(th)]]

        # (.4, 0., .2)로 이동합니다.
        T = np.identity(4)
        T[:3, 3] = [.4, 0., .2]

        # 카메라 각도를 현재 시간으로 설정합니다.
        camAng = t

        # 서로 다른 변환을 사용하여 장면을 렌더링합니다.
        # render(R, camAng)  # 회전 변환만 사용하여 렌더링합니다.
        # render(T, camAng)  # 이동 변환만 사용하여 렌더링합니다.

        # 결합된 변환 (1. 회전 후 2. 이동)으로 장면을 렌더링합니다.
        # render(T @ R, camAng)

        # 결합된 변환 (1. 이동 후 2. 회전)으로 장면을 렌더링합니다.
        # render(R @ T, camAng)

        # 지정된 창의 전면과 후면 버퍼를 스왑합니다.
        glfw.swap_buffers(window)

    # glfw 라이브러리를 종료합니다.
    glfw.terminate()


if __name__ == "__main__":
    main()
