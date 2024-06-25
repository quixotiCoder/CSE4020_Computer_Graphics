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
        inverseGamma = 1.0 / gamma
        self.color=np.power(self.color, inverseGamma)

    def toUINT8(self):
        return (np.clip(self.color, 0, 1)*255).astype(np.uint8)

class Camera:
    def __init__(self, viewPoint, viewDir, viewProjNormal, viewUp, viewWidth, viewHeight, projDistance):
        self.viewPoint = viewPoint
        self.viewDir = viewDir
        self.viewProjNormal = viewProjNormal
        self.viewUp = viewUp
        self.viewWidth = viewWidth
        self.viewHeight = viewHeight
        self.projDistance = projDistance

class Sphere:
    def __init__(self, center, radius, shader):
        self.center = center
        self.radius = radius
        self.shader = shader

class Light:
    def __init__(self, position, intensity):
        self.position = position
        self.intensity = intensity

class Shader:
    def __init__(self, type, diffuseColor):
        self.type = type
        self.diffuseColor = diffuseColor

class Lambertian(Shader):
    def __init__(self, type, diffuseColor):
        super().__init__(type, diffuseColor)

class Phong(Shader):
    def __init__(self, type, diffuseColor, specularColor, exponent):
        super().__init__(type, diffuseColor)
        self.specularColor = specularColor
        self.exponent = exponent

def normalize(vector):
    return vector / np.sqrt(vector @ vector)


def rayTracing(viewPoint, ori_d, surfaces):
    dots = -1
    intersect_distance = 99999

    hit_index = 0
    for surface in surfaces:
        d = normalize(ori_d)
        p = viewPoint - surface.center

        a = d @ d
        b = p @ d
        c = p @ p - (surface.radius ** 2)

        # 근의 공식 판별식
        result = b ** 2 - a * c

        if (result >= 0):
            t = -b + np.sqrt(result)
            if (t >= 0 and intersect_distance > t):
                intersect_distance = t
                dots = hit_index

            t = -b - np.sqrt(result)
            if (t >= 0 and intersect_distance > t):
                intersect_distance = t
                dots = hit_index

        hit_index += 1

    return dots, intersect_distance

def shading(viewPoint, ray, surfaces, lights, dots, distance):
    if (dots == -1):
        return Color(0., 0., 0.).toUINT8()

    else:
        R_k, G_k, B_k = 0., 0., 0.

        n = np.array([0., 0., 0.])
        unit_r = normalize(ray)
        v = -unit_r * distance

        n = normalize(viewPoint - v - surfaces[dots].center)

        for light in lights:
            l = normalize(v + light.position - viewPoint)

            hit_dots, temp = rayTracing(light.position, -l, surfaces)
            surface = surfaces[hit_dots]

            if (hit_dots == dots):
                if (surface.shader.type == "Lambertian"):
                    R_k += surface.shader.diffuseColor[0] * light.intensity[0] * max(0, (n @ l))
                    G_k += surface.shader.diffuseColor[1] * light.intensity[1] * max(0, (n @ l))
                    B_k += surface.shader.diffuseColor[2] * light.intensity[2] * max(0, (n @ l))

                elif (surface.shader.type == "Phong"):
                    unit_v = normalize(v)
                    h = (unit_v + l) / np.sqrt((unit_v + l) @ (unit_v + l))


                    R_k += (surface.shader.diffuseColor[0] * light.intensity[0] * max(0, (n @ l))) + surface.shader.specularColor[0] * light.intensity[0] * pow(max(0, (n @ h)), surface.shader.exponent[0])
                    G_k += (surface.shader.diffuseColor[1] * light.intensity[1] * max(0, (n @ l))) + surface.shader.specularColor[1] * light.intensity[1] * pow(max(0, (n @ h)), surface.shader.exponent[0])
                    B_k += (surface.shader.diffuseColor[2] * light.intensity[2] * max(0, (n @ l))) + surface.shader.specularColor[2] * light.intensity[2] * pow(max(0, (n @ h)), surface.shader.exponent[0])


        result = Color(R_k, G_k, B_k)
        result.gammaCorrect(2.2)
        return result.toUINT8()

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
        viewProjNormal = -1 * viewDir
        print('viewProjNormal', viewProjNormal)
        viewUp = np.array(c.findtext('viewUp').split()).astype(np.float64)
        print('viewUp', viewUp)
        viewWidth = np.array(c.findtext('viewWidth').split()).astype(np.float64)[0]
        print('viewWidth', viewWidth)
        viewHeight = np.array(c.findtext('viewHeight').split()).astype(np.float64)[0]
        print('viewHeight', viewHeight)
        if(c.findtext('projDistance')):
            projDistance = np.array(c.findtext('projDistance').split()).astype(np.float64)
            print('projDistance', projDistance)



    shaders = []
    for c in root.findall('shader'): # xml 속 <shader> 관련 정보 출력
        diffuseColor_c = np.array(c.findtext('diffuseColor').split()).astype(np.float64)
        type_c = c.get('type')

        if (type_c == 'Lambertian'):
            shaders.append(Lambertian(type_c, diffuseColor_c))
            print('name', c.get('name'))
            print('diffuseColor', diffuseColor_c)

        if (type_c == 'Phong'):
            specularColor_c = np.array(c.findtext('specularColor').split()).astype(np.float64)
            exponent_c = np.array(c.findtext('exponent').split()).astype(np.float64)
            shaders.append(Phong(type_c, diffuseColor_c, specularColor_c, exponent_c))
            print('name', c.get('name'))
            print('diffuseColor', diffuseColor_c)



    surfaces = []
    for index, c in enumerate(root.findall('surface')): # xml 속 <shape> 관련 정보 출력
        center_c = np.array(c.findtext('center').split()).astype(np.float64)
        radius_c = np.array(c.findtext('radius').split()).astype(np.float64)
        surfaces.append(Sphere(center_c, radius_c, shaders[index]))



    lights = []
    for c in root.findall('light'):
        position = np.array(c.findtext('position').split()).astype(np.float64)
        intensity = np.array(c.findtext('intensity').split()).astype(np.float64)
        lights.append(Light(position, intensity))
    # code.interact(local=dict(globals(), **locals()))


    # Create an empty image
    channels = 3
    img = np.zeros((imgSize[1], imgSize[0], channels), dtype=np.uint8)
    img[:, :] = 0

    cam = Camera(viewPoint, viewDir, viewProjNormal, viewUp, viewWidth, viewHeight, projDistance)

    w = cam.viewProjNormal
    u = np.cross(w, cam.viewUp)
    v = np.cross(w, u)

    unit_w = normalize(w)
    unit_u = normalize(u)
    unit_v = normalize(v)

    e = w - cam.projDistance * unit_w

    # Pixel-to-image mapping
    pixel_u = (cam.viewWidth / 2) * (1 / imgSize[0] + 1)
    pixel_v = (cam.viewHeight / 2) * (1 / imgSize[1] + 1)

    s = (cam.viewDir + e) + (pixel_u * unit_u) - (pixel_v * unit_v)

    pixel_x = cam.viewWidth / imgSize[0]
    pixel_y = cam.viewHeight / imgSize[1]

    for x in np.arange(imgSize[0]):
        for y in np.arange(imgSize[1]):
            ray = s - (pixel_x * x * unit_u) + (pixel_y * y * unit_v)

            dots, intersect_distance = rayTracing(cam.viewPoint, ray, surfaces)
            img[y][x] = shading(cam.viewPoint, ray, surfaces, lights, dots, intersect_distance)



    rawimg = Image.fromarray(img, 'RGB')
    # rawimg.save('out.png')
    rawimg.save(sys.argv[1] + '.png')

if __name__ == "__main__":
    main()