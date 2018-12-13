#!/usr/bin/env python

import numpy as np
import cv2

yellow = np.uint8([[[43,168,171]]])
hsv_yellow = cv2.cvtColor(yellow, cv2.COLOR_BGR2HSV)
print("Yellow")
print(hsv_yellow)

blue = np.uint8([[[169,0,0]]])
hsv_blue = cv2.cvtColor(blue, cv2.COLOR_BGR2HSV)
print("Blue")
print(hsv_blue)

red = np.uint8([[[17,18,164]]])
hsv_red = cv2.cvtColor(red, cv2.COLOR_BGR2HSV)
print("Red")
print(hsv_red)

green = np.uint8([[[41,169,41]]])
hsv_green = cv2.cvtColor(green, cv2.COLOR_BGR2HSV)
print("Green")
print(hsv_green)


"""
OUTPUT:  


Yellow
[[[ 29 191 171]]]
Blue
[[[120 255 169]]]
Red
[[[  0 229 164]]]
Green
[[[ 60 193 169]]]

Take [H-10, 100, 100] and [H-10, 255, 255] as lower and upper bounds, respectively 
"""
