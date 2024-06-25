#!/usr/bin/env python3
# -*- coding: utf-8 -*
# sample_python aims to allow seamless integration with lua.
# see examples below

# WARNING : This program only supports sphere rendering

import os
import sys
import pdb  # use pdb.set_trace() for debugging
import code  # or use code.interact(local=dict(globals(), **locals()))  for debugging.
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image


class Color:
    def __init__(self, R, G, B):
        self.color = np.array([R, G, B]).astype(np.float64)

    # Gamma corrects this color.
    # @param gamma the gamma value to use (2.2 is generally used).
    def gammaCorrect(self, gamma):
        inverseGamma = 1.0 / gamma
        self.color = np.power(self.color, inverseGamma)

    def toUINT8(self):
        return (np.clip(self.color, 0, 1) * 255).astype(np.uint8)


class Camera:
    def __init__(self, viewPoint, viewDir, viewProjNormal, viewUp, viewWidth, viewHeight, projDistance):
        self.viewPoint = viewPoint
        self.viewDir = viewDir
        self.viewProjNormal = viewProjNormal
        self.viewUp = viewUp
        self.viewWidth = viewWidth
        self.viewHeight = viewHeight
        self.projDistance = projDistance


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


class Sphere:
    def __init__(self, center, radius, shader):
        self.center = center
        self.radius = radius
        self.shader = shader


class Light:
    def __init__(self, position, intensity):
        self.position = position
        self.intensity = intensity


def normalize(vector):
    return vector / np.sqrt(vector @ vector)


def rayTracing(viewPoint, direction, figureList):
    near = -1
    nearistDistance = 99999

    index = 0
    for figure in figureList:

        direction = normalize(direction)
        p = viewPoint - figure.center

        # ax^2 + bx + c = 0
        a = direction @ direction
        b = p @ direction
        c = p @ p - (figure.radius ** 2)

        discriminant = b ** 2 - a * c
        # intersection has occuerd
        if (discriminant >= 0):
            t = -b + np.sqrt(discriminant)
            if (t >= 0 and nearistDistance > t):
                nearistDistance = t
                near = index

            t = -b - np.sqrt(discriminant)
            if (t >= 0 and nearistDistance > t):
                nearistDistance = t
                near = index

        index += 1

    return near, nearistDistance


def shading(viewPoint, ray, figureList, lightList, index, distance):

    if (index == -1):
        return Color(0., 0., 0.).toUINT8()
    else:
        # Colors
        R = 0.
        G = 0.
        B = 0.

        n = np.array([0., 0., 0.])
        rayUnit = normalize(ray)
        v = -rayUnit * distance


        n = normalize(viewPoint - v - figureList[index].center)

        for light in lightList:
            l = normalize(light.position - viewPoint + v)

            hit, dump = rayTracing(light.position, -l, figureList)
            figure = figureList[hit]

            # Check if the pixel receives light
            if (hit == index):
                # Lambertian Shading
                if (figure.shader.type == "Lambertian"):
                    R += figure.shader.diffuseColor[0] * light.intensity[0] * max(0, np.dot(n, l))
                    G += figure.shader.diffuseColor[1] * light.intensity[1] * max(0, np.dot(n, l))
                    B += figure.shader.diffuseColor[2] * light.intensity[2] * max(0, np.dot(n, l))

                # Phong Shading
                elif (figure.shader.type == "Phong"):
                    vUnit = normalize(v)
                    h = (vUnit + l) / np.sqrt((vUnit + l) @ (vUnit + l))

                    diffuse = []
                    phong = []
                    for i in range(3):
                        diffuse.append(figure.shader.diffuseColor[i] * light.intensity[i] * max(0, np.dot(n, l)))
                        phong.append(figure.shader.specularColor[i] * light.intensity[i] * pow(max(0, np.dot(n, h)),
                                                                                               figure.shader.exponent[0]))

                    R += diffuse[0] + phong[0]
                    G += diffuse[1] + phong[1]
                    B += diffuse[2] + phong[2]

        # return color of each pixel
        result = Color(R, G, B)
        result.gammaCorrect(2.2)
        return result.toUINT8()


def main():
    tree = ET.parse(sys.argv[1])
    root = tree.getroot()

    # set default values
    viewDir = np.array([0, 0, -1]).astype(np.float64)
    viewUp = np.array([0, 1, 0]).astype(np.float64)
    viewProjNormal = -1 * viewDir  # you can safely assume this. (no examples will use shifted perspective camera)
    viewWidth = 1.0
    viewHeight = 1.0
    projDistance = 1.0
    intensity = np.array([1, 1, 1]).astype(np.float64)  # how bright the light is.
    print(np.cross(viewDir, viewUp))

    imgSize = np.array(root.findtext('image').split()).astype(np.int32)

    # Get an information of the camera from the XML file
    for c in root.findall('camera'):
        viewPoint = np.array(c.findtext('viewPoint').split()).astype(np.float64)
        print('viewPoint', viewPoint)

        viewDir = np.array(c.findtext('viewDir').split()).astype(np.float64)
        print('viewDir', viewDir)

        viewProjNormal = -1 * viewDir
        print('viewProjNormal', viewProjNormal)

        viewUp = np.array(c.findtext('viewUp').split()).astype(np.float64)
        print('viewUp', viewUp)

        viewWidth = np.array(c.findtext('viewWidth').split()).astype(np.float64)
        print('viewWidth', viewWidth)

        viewHeight = np.array(c.findtext('viewHeight').split()).astype(np.float64)
        print('viewHieght', viewHeight)

        if (c.findtext('projDistance')):
            projDistance = np.array(c.findtext('projDistance').split()).astype(np.float64)
            print('projDistance', projDistance)

    # Set a camera
    camera = Camera(viewPoint, viewDir, viewProjNormal, viewUp, viewWidth, viewHeight, projDistance)

    shaderList = []
    for c in root.findall('shader'):
        diffuseColor_c = np.array(c.findtext('diffuseColor').split()).astype(np.float64)
        type_c = c.get('type')

        if (type_c == 'Lambertian'):
            shaderList.append(Lambertian(type_c, diffuseColor_c))

        elif (type_c == 'Phong'):
            specularColor_c = np.array(c.findtext('specularColor').split()).astype(np.float64)
            exponent_c = np.array(c.findtext('exponent').split()).astype(np.float64)
            shaderList.append(Phong(type_c, diffuseColor_c, specularColor_c, exponent_c))

        print('name', c.get('name'))
        print('type', type_c)
        print('diffuseColor', diffuseColor_c)

    figureList = []
    for index, c in enumerate(root.findall('surface')):
        center_c = np.array(c.findtext('center').split()).astype(np.float64)
        radius_c = np.array(c.findtext('radius').split()).astype(np.float64)
        figureList.append(Sphere(center_c, radius_c, shaderList[index]))


    lightList = []
    for c in root.findall('light'):
        position = np.array(c.findtext('position').split()).astype(np.float64)
        intensity = np.array(c.findtext('intensity').split()).astype(np.float64)
        lightList.append(Light(position, intensity))

    # code.interact(local=dict(globals(), **locals()))

    # Create an empty image
    channels = 3
    img = np.zeros((imgSize[1], imgSize[0], channels), dtype=np.uint8)
    img[:, :] = 0

    # Set vectors
    w = camera.viewProjNormal
    u = np.cross(w, camera.viewUp)
    v = np.cross(w, u)

    wUnit = normalize(w)
    uUnit = normalize(u)
    vUnit = normalize(v)

    # Find the vecter between an image plane and the origin
    e = w - camera.projDistance * wUnit

    # Compute s
    uInitial = (camera.viewWidth / 2) * (1 / imgSize[0] + 1)
    vInitial = (camera.viewHeight / 2) * (1 / imgSize[1] + 1)
    s = (camera.viewDir + e) + (uInitial * uUnit) - (vInitial * vUnit)

    px = camera.viewWidth / imgSize[0]
    py = camera.viewHeight / imgSize[1]

    # Raytracing & Shading
    for x in np.arange(imgSize[0]):
        for y in np.arange(imgSize[1]):
            # moving ray
            ray = s - (px * x * uUnit) + (py * y * vUnit)

            near, nearistDistance = rayTracing(camera.viewPoint, ray, figureList)
            img[y][x] = shading(camera.viewPoint, ray, figureList, lightList, near, nearistDistance)

    rawimg = Image.fromarray(img, 'RGB')
    # rawimg.save('out.png')
    rawimg.save(sys.argv[1] + '.png')


if __name__ == "__main__":
    main()

    '''# Find the vecter between an image plane and the origin
    imageVector = w - camera.projDistance * wUnit

    # Compute s
    uInitial = (camera.viewWidth / 2) * (1 / imgSize[0] + 1)
    vInitial = (camera.viewHeight / 2) * (1 / imgSize[1] + 1)
    s = (camera.viewDir + imageVector) + (uInitial * uUnit) - (vInitial * vUnit)

    px = camera.viewWidth / imgSize[0]
    py = camera.viewHeight / imgSize[1]

    # Raytracing & Shading
    for x in np.arange(imgSize[0]):
        for y in np.arange(imgSize[1]):
            # moving ray
            ray = s - (px * x * uUnit) + (py * y * vUnit)

            near, nearistDistance = rayTracing(camera.viewPoint, ray, figureList)
            img[y][x] = shading(camera.viewPoint, ray, figureList, lightList, near, nearistDistance)

    rawimg = Image.fromarray(img, 'RGB')
    # rawimg.save('out.png')
    rawimg.save(sys.argv[1] + '.png')


if __name__ == "__main__":
    main()'''