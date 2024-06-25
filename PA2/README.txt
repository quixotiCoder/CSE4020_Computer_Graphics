# bezier 등에서 활용
from math import atan2 

# PA2용 변수 추가
# pa2
click_cnt = 0
pos_of_click = None
pos_of_cow = [None for i in range(6)]

isStart = False
startTime = 0
prev_cow = None

# display 함수에 클릭 수에 따른 cow 저장 + 세부 사항 별도 함수
    if click_cnt <= 6:
        drawCowsBeforeAnimation()
    elif click_cnt > 6:
        animateCow()

    glFlush()

def drawCowsBeforeAnimation()
def animateCow()
def initAnimation()
def updateCowPosition(animTime)
def getControlPoints(segment_index)
def computeCatmullRom(P0, P1, P2, P3, t)
def resetAnimation()

# bezier 함수 미분 -> cow head
def bezier_derivative(P0, P1, P2, P3, t):
    vec = (-3 * (1 - t) ** 3 * P0 + (9 * t ** 2 - 12 * t) * P1 + (-9 * t ** 2 + 6 * t + 3) * P2 + 3 * t ** 3 * P3) / 6
    return vec

# cow head 방향 컨트롤 by pitch->roll->yaw
def control_cow_head(P0, P1, P2, P3, t)

# 마우스 click 버튼 이벤트 처리 추가
def onMouseButton(window,button, state, mods):
    global isDrag, click_cnt, V_DRAG, H_DRAG, cow2wld, pos_of_click
    GLFW_DOWN = 1;
    GLFW_UP = 0;
    x, y = glfw.get_cursor_pos(window)
    if button == glfw.MOUSE_BUTTON_LEFT:
        if state == GLFW_DOWN:
            isDrag = V_DRAG;
            pos_of_click = (x, y);
            print("Left mouse down-click at %d %d\n" % (x, y))
            # start vertical dragging
        elif state == GLFW_UP and isDrag != 0:
            if click_cnt == 0:
                isDrag = H_DRAG;
                click_cnt += 1;
            elif (x, y) != pos_of_click:
                # v drag happend
                isDrag = H_DRAG;
            elif isDrag != 0:
                if (click_cnt > 6):
                    isDrag = 0;
                    # click_cnt = 0;
                else:
                    pos_of_cow[click_cnt - 1] = cow2wld.copy();
                    print(click_cnt, cow2wld)
                    click_cnt += 1;
                    isDrag = H_DRAG;
            print("Left mouse up\n");
            # start horizontal dragging using mouse-move events.
    elif button == glfw.MOUSE_BUTTON_RIGHT:
        if state == GLFW_DOWN:
            print("Right mouse click at (%d, %d)\n" % (x, y));

# 마우스 drag 이벤트 처리 추가
def onMouseDrag(window, x, y):
    global isDrag, cursorOnCowBoundingBox, pickInfo, cow2wld, pos_of_click
    if isDrag:
        print("in drag mode %d\n" % isDrag)
        ray = screenCoordToRay(window, x, y)
        pp = pickInfo

        if isDrag == V_DRAG:
            # vertical dragging
            # TODO:
            # create a dragging plane perpendicular to the ray direction,
            # and test intersection with the screen ray.
            if cursorOnCowBoundingBox:
                p = Plane(np.array((1, 0, 0)), pp.cowPickPosition)
                c = ray.intersectsPlane(p)
                currentPos = ray.getPoint(c[1])
                currentPos[0], currentPos[2] = pp.cowPickPosition[0], pp.cowPickPosition[2]
        else:
            # horizontal dragging
            if cursorOnCowBoundingBox:
                p = Plane(np.array((0, 1, 0)), pp.cowPickPosition)
                c = ray.intersectsPlane(p)
                currentPos = ray.getPoint(c[1])

        if cursorOnCowBoundingBox:
            T = np.eye(4)
            setTranslation(T, currentPos - pp.cowPickPosition)
            cow2wld = T @ pp.cowPickConfiguration

            cowPickPosition = currentPos
            cowPickLocalPos = transform(np.linalg.inv(cow2wld), cowPickPosition)
            pickInfo = PickInfo(c[1], cowPickPosition, cow2wld, cowPickLocalPos)
    else:
        ray = screenCoordToRay(window, x, y)
        planes = []
        cow = cowModel
        bbmin = cow.bbmin
        bbmax = cow.bbmax

        planes.append(makePlane(bbmin, bbmax, vector3(0, 1, 0)))
        planes.append(makePlane(bbmin, bbmax, vector3(0, -1, 0)))
        planes.append(makePlane(bbmin, bbmax, vector3(1, 0, 0)))
        planes.append(makePlane(bbmin, bbmax, vector3(-1, 0, 0)))
        planes.append(makePlane(bbmin, bbmax, vector3(0, 0, 1)))
        planes.append(makePlane(bbmin, bbmax, vector3(0, 0, -1)))

        o = ray.intersectsPlanes(planes)
        cursorOnCowBoundingBox = o[0]
        cowPickPosition = ray.getPoint(o[1])
        cowPickLocalPos = transform(np.linalg.inv(cow2wld), cowPickPosition)
        pickInfo = PickInfo(o[1], cowPickPosition, cow2wld, cowPickLocalPos)d, cowPickLocalPos)
