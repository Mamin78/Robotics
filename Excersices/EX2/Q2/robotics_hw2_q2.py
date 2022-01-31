# -*- coding: utf-8 -*-
"""Robotics_HW2_Q2.ipynb

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/11OPvl3rCfOWes8p2bGIXguQRmnbgkQ1e
"""

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy import signal

myimg = Image.open(r'/content/drive/MyDrive/Colab Notebooks/Robotics/70.png')
myimg

gaussian_kernel = ( 1/16 ) * np.array([
                [1,2,1],
                [2,4,2],
                [1,2,1],
                ])

prewitt_kernel_hor = np.array([
                [-1,-1,-1],
                [ 0, 0, 0],
                [ 1, 1, 1],
                ])
prewitt_kernel_ver = np.array([
               [-1,0,1],
               [-1,0,1],
               [-1,0,1]
               ])

r, g, b = myimg.convert('RGB').split()
imgr = np.asarray(r)
imgg = np.asarray(g)
imgb = np.asarray(b)

rnormalized = np.uint8(signal.convolve2d(imgr, gaussian_kernel , boundary='fill', mode='same').astype(int))
gnormalized = np.uint8(signal.convolve2d(imgg, gaussian_kernel , boundary='fill', mode='same').astype(int))
bnormalized = np.uint8(signal.convolve2d(imgb, gaussian_kernel , boundary='fill', mode='same').astype(int))

normalrgb = np.dstack((rnormalized, gnormalized, bnormalized))
normal_img = Image.fromarray(normalrgb)
normal_img

grayimg = normal_img.convert("L")
grayimgarr = np.asarray(grayimg)

graynormalized = signal.convolve2d(grayimgarr, gaussian_kernel , boundary='fill', mode='same').astype(int)

horizontal_edges = signal.convolve2d(graynormalized, prewitt_kernel_hor, boundary='fill', mode='same')
vertical_edges = signal.convolve2d(graynormalized, prewitt_kernel_ver, boundary='fill', mode='same')

def TransformTo_0_255(vector):
  v = []
  transformer = lambda x: 0 if x <0 else (255 if x>255 else x) 
  for row in vector:
    newrow = np.array([transformer(xi) for xi in row])
    v.append(newrow)
  return np.array(v)

def NormalizeTO_0_255(vector):
  max , min  = np.amax(vector) , np.amin(vector)
  return np.uint8( ( vector - min) * 255 / (max - min) )

fig, (ax_orig, ax_norm, ax_edge_hor, ax_edge_ver) = plt.subplots(4, 1, figsize=(40, 100))

ax_orig.imshow( grayimgarr, cmap='gray')
ax_orig.set_title('Original')
ax_orig.set_axis_off()

ax_norm.imshow( graynormalized, cmap='gray')
ax_norm.set_title('normalized')
ax_norm.set_axis_off()

ax_edge_hor.imshow(horizontal_edges, cmap='gray')
ax_edge_hor.set_title('horizontal_edges')
ax_edge_hor.set_axis_off()

ax_edge_ver.imshow( vertical_edges , cmap='gray')
ax_edge_ver.set_title('vertical_edges')
ax_edge_ver.set_axis_off()
fig.show()

fig, (ax_orig, ax_norm, ax_edge_hor, ax_edge_ver) = plt.subplots(4, 1, figsize=(40, 100))

ax_orig.imshow( grayimgarr, cmap='gray')
ax_orig.set_title('Original')
ax_orig.set_axis_off()

ax_norm.imshow( np.absolute(graynormalized), cmap='gray')
ax_norm.set_title('normalized')
ax_norm.set_axis_off()

ax_edge_hor.imshow(np.absolute(horizontal_edges), cmap='gray')
ax_edge_hor.set_title('horizontal_edges')
ax_edge_hor.set_axis_off()

ax_edge_ver.imshow( np.absolute(vertical_edges) , cmap='gray') 
ax_edge_ver.set_title('vertical_edges')
ax_edge_ver.set_axis_off()
fig.show()

fig, (ax_orig, ax_norm, ax_edge_hor, ax_edge_ver) = plt.subplots(4, 1, figsize=(40, 100))

ax_orig.imshow( grayimgarr, cmap='gray')
ax_orig.set_title('Original')
ax_orig.set_axis_off()

ax_norm.imshow(horizontal_edges, cmap='gray')
ax_norm.set_title('horizontal_edges')
ax_norm.set_axis_off()

ax_edge_hor.imshow(TransformTo_0_255(horizontal_edges), cmap='gray')
ax_edge_hor.set_title('Transformed horizontal_edges')
ax_edge_hor.set_axis_off()

ax_edge_ver.imshow( NormalizeTO_0_255(horizontal_edges), cmap='gray')
ax_edge_ver.set_title('normalized vertical_edges')
ax_edge_ver.set_axis_off()
fig.show()

fig, (ax_orig, ax_norm, ax_edge_hor, ax_edge_ver) = plt.subplots(4, 1, figsize=(40, 100))

ax_orig.imshow( grayimgarr, cmap='gray')
ax_orig.set_title('Original')
ax_orig.set_axis_off()

ax_norm.imshow(vertical_edges, cmap='gray')
ax_norm.set_title('vertical_edges')
ax_norm.set_axis_off()

ax_edge_hor.imshow(TransformTo_0_255(vertical_edges), cmap='gray')
ax_edge_hor.set_title('Transformed vertical_edges')
ax_edge_hor.set_axis_off()

ax_edge_ver.imshow( NormalizeTO_0_255(vertical_edges), cmap='gray')
ax_edge_ver.set_title('normalized vertical_edges')
ax_edge_ver.set_axis_off()
fig.show()

grayimg = myimg.convert("L")
grayimgarr = np.asarray(grayimg)

graynormalized = signal.convolve2d(grayimgarr, gaussian_kernel , boundary='fill', mode='same').astype(int)

horizontal_edges = signal.convolve2d(graynormalized, prewitt_kernel_hor, boundary='fill', mode='same')

vertical_edges = signal.convolve2d(graynormalized, prewitt_kernel_ver, boundary='fill', mode='same')

fig, (ax_orig, ax_norm, ax_edge_hor, ax_edge_ver) = plt.subplots(4, 1, figsize=(40, 100))

ax_orig.imshow( grayimg, cmap='gray')
ax_orig.set_title('Original')
ax_orig.set_axis_off()

ax_norm.imshow(graynormalized, cmap='gray')
ax_norm.set_title('normalized')
ax_norm.set_axis_off()

ax_edge_hor.imshow(horizontal_edges, cmap='gray')
ax_edge_hor.set_title('horizontal_edges')
ax_edge_hor.set_axis_off()

ax_edge_ver.imshow( vertical_edges , cmap='gray')
ax_edge_ver.set_title('vertical_edges')
ax_edge_ver.set_axis_off()
fig.show()

fig, (ax_orig, ax_norm, ax_edge_hor, ax_edge_ver) = plt.subplots(4, 1, figsize=(40, 100))

ax_orig.imshow( grayimg, cmap='gray')
ax_orig.set_title('Original')
ax_orig.set_axis_off()

ax_norm.imshow(np.absolute(graynormalized), cmap='gray')
ax_norm.set_title('normalized')
ax_norm.set_axis_off()


ax_edge_hor.imshow(np.absolute(horizontal_edges), cmap='gray')
ax_edge_hor.set_title('horizontal_edges')
ax_edge_hor.set_axis_off()


ax_edge_ver.imshow( np.absolute(vertical_edges) , cmap='gray') 
ax_edge_ver.set_title('vertical_edges')
ax_edge_ver.set_axis_off()
fig.show()

fig, (ax_orig, ax_norm, ax_edge_hor, ax_edge_ver) = plt.subplots(4, 1, figsize=(40, 100))

ax_orig.imshow( grayimg, cmap='gray')
ax_orig.set_title('Original')
ax_orig.set_axis_off()

ax_norm.imshow(horizontal_edges, cmap='gray')
ax_norm.set_title('horizontal_edges')
ax_norm.set_axis_off()

ax_edge_hor.imshow(TransformTo_0_255(horizontal_edges), cmap='gray')
ax_edge_hor.set_title('Transformed horizontal_edges')
ax_edge_hor.set_axis_off()

ax_edge_ver.imshow( NormalizeTO_0_255(horizontal_edges), cmap='gray')
ax_edge_ver.set_title('normalized vertical_edges')
ax_edge_ver.set_axis_off()
fig.show()

fig, (ax_orig, ax_norm, ax_edge_hor, ax_edge_ver) = plt.subplots(4, 1, figsize=(40, 100))

ax_orig.imshow( grayimg, cmap='gray')
ax_orig.set_title('Original')
ax_orig.set_axis_off()

ax_norm.imshow(vertical_edges, cmap='gray')
ax_norm.set_title('vertical_edges')
ax_norm.set_axis_off()

ax_edge_hor.imshow(TransformTo_0_255(vertical_edges), cmap='gray')
ax_edge_hor.set_title('Transformed vertical_edges')
ax_edge_hor.set_axis_off()

ax_edge_ver.imshow( NormalizeTO_0_255(vertical_edges), cmap='gray')
ax_edge_ver.set_title('normalized vertical_edges')
ax_edge_ver.set_axis_off()
fig.show()