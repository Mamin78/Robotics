# -*- coding: utf-8 -*-
"""Robotics_EX2_extra

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/14b-svMfd2P2zCgvJF4hGPR-W05u8Hzaw

# **Reading the data**
"""

def Convert(string):
  string = string[1:-1]
  li = list(string.split(","))
  li = [float(i) for i in li]

  return li

data = []
for i in range(1, 9):
  f = open("ds"+str(i)+".txt", "r")
  data.extend(Convert(f.read()))

len(data)

data

"""# **a)**"""

import math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.pyplot import figure

x = []
y = []

d = 0
for i in range(len(data)):
  if data[i] > 500 or data[i] < -500:
    continue
  x.append((data[i] / 2000) * math.cos(math.radians(i * 1.4)))
  y.append((data[i] / 2000) * math.sin(math.radians(i * 1.4)))
  d += 1.4

figure(figsize=(8, 6), dpi=80)

plt.scatter(x = x, y = y)
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

"""# **b)**"""

x = []
y = []

d = 0
for i in range(len(data)):
  if data[i] > 500 or data[i] < -500 or d > 90:
    continue
  x.append((data[i] / 2000) * math.cos(math.radians(i * 1.4)))
  y.append((data[i] / 2000) * math.sin(math.radians(i * 1.4)))
  d += 1.4

X_ps = np.array([x, y])
X_ps = X_ps.T
c = np.ones((len(x), 1))

def fit_line(X, c):
  s = np.dot(X.T, X)
  si = np.linalg.inv(s)
  sx = np.dot(si, X.T)
  return np.dot(sx, c)

a, b = fit_line(X_ps, c)
print(a, b)

def line(a, b, x):
  return (1 - a * x ) / b

fx, fy = max(x), line(a, b, max(x))
sx, sy = min(x), line(a, b, min(x))

print(fx, fy)
print(sx, sy)

figure(figsize=(8, 6), dpi=80)

plt.scatter(x = x, y = y)
plt.plot([fx, sx], [sy, fy], 'ro-')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

"""# **c)**"""

import random

def select_indexes(maxi, all_i):
  maxi -= 1
  f = random.randint(0, maxi)
  s = random.randint(0, maxi)
  
  while (f, s) in all_i or (s, f) in all_i or s == f:
      f = random.randint(0, maxi)
      s = random.randint(0, maxi)
  all_i.append((s, f))

  return s, f

def get_line_between_two_points(xf, xs, yf, ys):
  a = (ys - yf) / (xs - xf)
  c = yf - a * xf
  return a, c

def distance_line_point(a, b, c, x, y):
  x = np.array(x)
  y = np.array(y)
  abs_ = abs(a * x + b * y + c)
  dis = abs_ / math.sqrt(a * a + b * b)
  return dis.tolist()

def get_inliers(dis, threshold):
  inliers = []
  for i in range(len(dis)):
    if dis[i] < threshold:
      inliers.append(i)
  return inliers, len(inliers)

def ransac(X, Y, threshold, itr = 100000):
  all_dis = {}
  all_inliers = {}
  all_lines = {}

  all_i = []

  max_k = -1
  max_arg = 0

  for i in range(itr):
    fi, si = select_indexes(len(X), all_i)
    
    a, c = get_line_between_two_points(x[fi], x[si], y[fi], y[si])
    dis = distance_line_point(a, -1, c, x, y)
    inliers, n_inliers = get_inliers(dis, threshold)

    all_dis[i] = dis
    all_inliers[i] = inliers
    all_lines[i] = [a, c, fi, si]

    if n_inliers > max_k:
      max_k = n_inliers
      max_arg = i
      
    return all_lines[max_arg]

best_line = ransac(x, y, 0.001)

def get_y(a, c, x):
  return a * x + c

fx, fy = max(x), get_y(best_line[0], best_line[1], max(x))
sx, sy = min(x), get_y(best_line[0], best_line[1], min(x))

figure(figsize=(8, 6), dpi=80)

plt.scatter(x = x, y = y)
plt.plot([fx, sx], [sy, fy], 'ro-')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

"""# **d)**"""

x = []
y = []
for i in range(len(data)):
  if data[i] > 500 or data[i] < -500:
    continue
  x.append((data[i] / 2000) * math.cos(math.radians(i * 1.4)))
  y.append((data[i] / 2000) * math.sin(math.radians(i * 1.4)))

figure(figsize=(8, 6), dpi=80)

plt.scatter(x = x, y = y)
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

def get_max_dis(X, Y, a, c):
  max_d = -1
  max_arg = 0

  dis = distance_line_point(a, -1, c, X, Y)
  for i in range(len(X)):
    if dis[i] > max_d:
      max_d = dis[i]
      max_arg = i
  return max_arg, max_d

breaks = set()
def split_merge(X, Y, threshold=0.0025, start = 0, end = len(x) - 1):
  a_len = end - start
  if a_len < 2:
    return
  a, c = get_line_between_two_points(X[start], X[end], Y[start], Y[end])

  max_arg, max_d = get_max_dis(X[start:end], Y[start:end], a, c)
  max_arg += start

  # print("length is: ", a_len)
  # print(max_arg, max_d)
  if max_arg == start or max_arg == a_len:
    return
  if max_d > threshold:
    # print("TRUE", max_arg)
    breaks.add(max_arg)
    split_merge(X, Y, threshold, start, end= max_arg)
    split_merge(X, Y, threshold, start = max_arg + 1, end = end)
  return

split_merge(x, y)

def get_sum_dis(X, Y, a, c):
  dis = distance_line_point(a, -1, c, X, Y)
  return np.sum(dis)

breaks = list(breaks)
breaks.sort()

def creat_stack(breaks):
  sb = [breaks[i] for i in range(1, len(breaks))]
  return breaks[0], sb

def get_line_length(s, e):
  return math.sqrt(((x[e] - x[s]) * (x[e] - x[s])) + (y[e] - y[s]) * (y[e] - y[s]))

def MNE(s, e):
  l = get_line_length(s, e)

  a, c = get_line_between_two_points(x[s], x[e], y[s], y[e])

  max_arg, max_d = get_max_dis(x[s:e], y[s:e], a, c)

  return  max_d / l

# def MNE(s, e):
#   l = get_line_length(s, e)

#   a, c = get_line_between_two_points(x[s], x[e], y[s], y[e])

#   # max_arg, max_d = get_max_dis(x[s:e], y[s:e], a, c)

#   return  get_sum_dis(x[s:e], y[s:e], a, c) / l

def merge_part(breaks):
  start = 0
  mid, lb = creat_stack(breaks)
  bp = [i for i in breaks]
  i = 0

  print(len(lb))
  while i<len(lb):
    # print('i is: ', i)
    if MNE(start, lb[i]) < (MNE(start, mid) + MNE(mid, lb[i])) / 2:
      bp.remove(mid)
      mid = lb[i]
      # print('first')
    
    elif (i > len(lb) - 1) and (MNE(mid, lb[i+1]) < (MNE(mid, lb[i]) + MNE(lb[i], lb[i+1])) / 2):
      # print('second')
      bp.remove(lb[i])
      start = mid
      mid = lb[i+1]
      i += 1

    else:
      # print('third')
      if (i>= len(lb) - 1):
        break
      start = lb[i]
      mid = lb[i+1]
      i+=1
    i += 1
  
  return bp

tmp = merge_part(breaks)

breaks = list(breaks)
breaks.sort()

figure(figsize=(8, 6), dpi=80)

plt.scatter(x = x, y = y)
prev = 0
for i in breaks:
  plt.plot([x[prev], x[i]], [y[prev], y[i]], 'ro-')
  prev = i

plt.plot([x[prev], x[-1]], [y[prev], y[-1]], 'ro-')

plt.xlabel('X')
plt.ylabel('Y')
plt.show()

tmp = list(tmp)
tmp.sort()

figure(figsize=(8, 6), dpi=80)
plt.scatter(x = x, y = y)
prev = 0
for i in tmp:
  plt.plot([x[prev], x[i]], [y[prev], y[i]], 'ro-')
  prev = i

plt.plot([x[prev], x[-1]], [y[prev], y[-1]], 'ro-')

plt.xlabel('X')
plt.ylabel('Y')
plt.show()