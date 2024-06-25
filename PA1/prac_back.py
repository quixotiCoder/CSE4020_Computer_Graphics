#!/usr/bin/env python3
# -*- coding: utf-8 -*
# sample_python aims to allow seamless integration with lua.
# see examples below

import os
import sys
import pdb  # use pdb.set_trace() for debugging
import code # or use code.interact(local=dict(globals(), **locals()))  for debugging.
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image

class Color:
    def __init__(self, R, G, B):
        self.color=np.array([R,G,B]).astype(np.float64)

    # Gamma corrects this color.
    # @param gamma the gamma value to use (2.2 is generally used).
    def gammaCorrect(self, gamma):
        inverseGamma = 1.0 / gamma;
        self.color=np.power(self.color, inverseGamma)

    def toUINT8(self):
        return (np.clip(self.color, 0,1)*255).astype(np.uint8)


def main():
    tree = ET.parse(sys.argv[1])
    root = tree.getroot()

    # xml 속 <camera>의 카메라 변수 초기화
    viewPoint = np.array([0, 0, 0]).astype(np.float64)
    viewDir = np.array([0, 0, -1]).astype(np.float64)
    viewProjNormal = -1 * viewDir
    viewUp = np.array([0, 1, 0]).astype(np.float64)
    projDistance = 1.0
    viewWidth = 1.0
    viewHeight = 1.0

    # xml 속 <light> 변수 초기화
    light_position = np.array([0, 0, 0]).astype(np.float64)
    intensity = np.array([1, 1, 1]).astype(np.float64)

    # xml 속 <surface type="Sphere"> 변수 초기화
    center = np.array([1, 1, 1]).astype(np.float64)
    radius = 0

    print(np.cross(viewDir, viewUp))

    imgSize = np.array(root.findtext('image').split()).astype(np.int32)

    for c in root.findall('camera'): # xml 속 <camera> 관련 정보 출력
        viewPoint = np.array(c.findtext('viewPoint').split()).astype(np.float64)
        print('viewpoint', viewPoint)
        viewDir = np.array(c.findtext('viewDir').split()).astype(np.float64)
        print('viewDir', viewDir)
        projNormal = np.array(c.findtext('projNormal').split()).astype(np.float64)
        print('projNormal', projNormal)
        viewUp = np.array(c.findtext('viewUp').split()).astype(np.float64)
        print('viewUp', viewUp)
        viewWidth = np.array(c.findtext('viewWidth').split()).astype(np.float64)[0]
        print('viewWidth', viewWidth)
        viewHeight = np.array(c.findtext('viewHeight').split()).astype(np.float64)[0]
        print('viewHeight', viewHeight)
        projDistance = np.array(c.findtext('projDistance').split()).astype(np.float64)[0]
        print('projDistance', projDistance)


    shader = [] # xml 속 <shader>의 type 담기
    for c in root.findall('shader'): # xml 속 <shader> 관련 정보 출력
        diffuseColor_c = np.array(c.findtext('diffuseColor').split()).astype(np.float64)
        if c.get('type') == "Lambertian":
            shader.append((c.get('name'), diffuseColor_c, c.get('type')))
            print('name', c.get('name'))
            print('diffuseColor', diffuseColor_c)

        elif c.get('type') == "Phong":
            exponent = np.array(c.findtext('exponent').split()).astype(np.int32)[0] # phong 용 exponent 처리 in 'one-sphere'
            specularColor = np.array(c.findtext('specularColor').split()).astype(np.float64)
            shader.append((c.get('name'), diffuseColor_c, c.get('type'), specularColor, exponent))
            print('name', c.get('name'))
            print('diffuseColor', diffuseColor_c)

    surface = [] # xml 속 <surface>의 type 담기
    for c in root.findall('surface'):
        center_c = np.array(c.findtext('center').split()).astype(np.float64)
        radius_c = np.array(c.findtext('radius').split()).astype(np.float64)
        figureList.append(Sphere(center_c, radius_c, shaderList[cnt]))

      # lambertian shading

    def lamb_shading(kd, I, n, l):
        m = max(0, np.dot(n, l))
        distancem = np.array([kd[0] * I[0], kd[1] * I[1], kd[2] * I[2]])
        distance = distancem * m
        return distance

        # blinn_phong shading

    def phong_shading(ks, I, v, l, n):
        h = (v + l) / np.sqrt(np.dot(v + l, v + l))
        m = max(0, np.dot(n, h)) ** exponent
        ls = np.array([ks[0] * I[0], ks[1] * I[1], ks[2] * I[2]]) * m
        return ls

        # camera frame coordination

    vec_u = unit_dir(np.cross(viewDir, viewUp))
    vec_w = (-1) * unit_dir(viewDir)
    vec_v = unit_dir(np.cross(vec_w, vec_u))

    # world frame coordination through viewPoint p on image plane
    def make_s(i, j):
        coord_u = (-1) * viewWidth / 2 + viewWidth * (i + 0.5) / imgSize[0]
        coord_v = viewHeight / 2 - viewHeight * (j + 0.5) / imgSize[1]
        s = viewPoint + coord_u * vec_u + coord_v * vec_v - projDistance * vec_w
        return s

        # |p+td-c|=radius

    # p+td = ray, c = sphere center
    def intersect_distance(p, unit_d, sphere_center, sphere_radius):
        cp_point = p - sphere_center  # OP - OC = CP
        discriminant = (np.dot(unit_d, cp_point)) ** 2 - np.dot(cp_point, cp_point) + sphere_radius ** 2
        if discriminant >= 0:
            t1 = (-1) * np.dot(unit_d, cp_point) - np.sqrt(discriminant)
            t2 = (-1) * np.dot(unit_d, cp_point) + np.sqrt(discriminant)
            if t1 >= 0:
                if t2 >= 0:
                    res = t1 if t1 <= t2 else t2
                else:
                    res = t1
                return res
            elif t2 >= 0:
                return t2
            else:
                return 99999999
        else:
            return 99999999

    def find_min(d_list):
        min = d_list[0]
        dots = 0

        for i in range(len(d_list)):
            if min > d_list[i]:
                min = d_list[i]
                dots = i
        return (dots, min)

        # image plane 의 ray center in world frame

    ray = viewPoint + unit_dir(viewDir)
    print(f'ray ray is {ray}')

    # Create an empty image
    channels = 3
    img = np.zeros((imgSize[1], imgSize[0], channels), dtype=np.uint8)
    img[:, :] = 0

    # main rendering
    for i in np.arange(imgSize[0]):
        for j in np.arange(imgSize[1]):

            s = make_s(i, j)
            imagevector = unit_dir(s - viewPoint)  # OS - OP = PS

            view_distance = []  # view_distance에 대한 각주
            light_distance = []  # light_distance에 대한 각주

            # 눈에 보이는 도형 찾기
            for sphere in surface:  # surface에 대한 각주
                vd = intersect_distance(viewPoint, imagevector, sphere[1], sphere[2])
                view_distance.append(vd)
            vdots, vmin = find_min(view_distance)

            # 보이는 구가 없는 경우
            if vmin == 99999999:
                gam_color = Color(0, 0, 0)
                gam_color.gammaCorrect(2.2)
                img[j][i] = gam_color.toUINT8()

                # 보이는 구가 있는 경우
            else:
                # image plane 너머의 공간 상의 점 T (구 위의 점)
                vec_ot = viewPoint + vmin * imagevector
                dir_ct = unit_dir(vec_ot - center)
                dir_lt = unit_dir(vec_ot - light_position)  # OT - OL = LT

                # 빛 받는 도형 찾기
                for sphere in surface:
                    distance = intersect_distance(light_position, dir_lt, sphere[1], sphere[2])
                    light_distance.append(distance)
                ldots = find_min(light_distance)[0]

                sphere = surface[vdots]
                kd = sphere[0]
                center = sphere[1]
                radius = sphere[2]
                typ = sphere[3]
                v = -imagevector

                n = dir_ct
                l = -dir_lt

                # 보이는 구가 빛을 받는 경우
                if vdots == ldots:
                    clr = lamb_shading(kd, intensity, n, l)
                    if typ == "Phong":
                        ks = sphere[4]
                        p_clr = phong_shading(ks, intensity, v, l, n)
                        clr += p_clr
                    gam_color = Color(clr[0], clr[1], clr[2])
                    gam_color.gammaCorrect(2.2)
                    img[j][i] += gam_color.toUINT8()

                    # 보이는 구가 빛을 받지 못하는 경우
                elif vdots != ldots:
                    gam_color = Color(0, 0, 0)
                    gam_color.gammaCorrect(2.2)
                    img[j][i] += gam_color.toUINT8()

    rawimg = Image.fromarray(img, 'RGB')
    rawimg.save(sys.argv[1] + '.png')

if __name__ == "__main__":
    main()
