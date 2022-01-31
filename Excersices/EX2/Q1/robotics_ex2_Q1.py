# -*- coding: utf-8 -*-
"""Robotics_EX2

Automatically generated by Colaboratory.

Original file is located at
    https://colab.research.google.com/drive/1Ijdma0OhCNgItyr4mu1F5DLggTs5bIv6

# **a)**
"""

import math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.pyplot import figure

inf = 0
points = [0.5201594829559326, 0.5202369689941406, 0.42739346623420715, 0.4418208599090576, 0.5507082343101501, 0.4677025079727173, 0.46121981739997864, 0.5288999080657959, 0.5168297290802002, 0.5362252593040466, 0.49491605162620544, 0.41996413469314575, 0.4715176522731781, 0.5633909702301025, 0.43786174058914185, 0.5142707228660583, 0.41325870156288147, 0.5400388240814209, 0.5178366303443909, 0.5347734093666077, 0.5014176964759827, 0.5332620143890381, 0.5744584798812866, 0.5030121803283691, 0.5136361122131348, 0.4206880033016205, 0.5641984939575195, 0.5860626101493835, 0.391154021024704, 0.5938740372657776, 0.5031955242156982, 0.4587433934211731, 0.45114344358444214, 0.612488329410553, 0.5149634480476379, 0.5976732969284058, 0.5071269273757935, 0.5828769207000732, 0.5384292006492615, 0.6269007921218872, 0.5506464242935181, 0.6788175106048584, 0.624706506729126, 0.6597438454627991, 0.6259024143218994, 0.6821727752685547, 0.7030988931655884, 0.21206599473953247, 0.23518238961696625, 0.35561665892601013, 0.24944216012954712, 0.3193572461605072, 0.2009689211845398, 0.32280072569847107, 0.335549920797348, 0.19971971213817596, 0.3089701533317566, 0.2840210199356079, 0.17885364592075348, 0.2788484990596771, 0.23741191625595093, 0.13557519018650055, 0.23521356284618378, 0.2557879090309143, 0.17776194214820862, 0.23630499839782715, 0.11543351411819458, 0.1892642378807068, inf, 0.2158263921737671, 0.2138580083847046, 0.18487036228179932, 0.14990663528442383, 0.1338089406490326, 0.15677200257778168, 0.30763113498687744, 0.21442481875419617, 0.14488066732883453, 0.11776462197303772, 0.3299056887626648, 0.2599114775657654, 0.13347584009170532, 0.13925665616989136, 0.21161971986293793, 0.16282959282398224, 0.20783330500125885, 0.19382697343826294, 0.20569825172424316, 0.21906113624572754, 0.2831454873085022, 0.22542957961559296, inf, 0.2138456404209137, 0.21625907719135284, 0.1607387810945511, 0.25801578164100647, 0.1943732649087906, 0.2283838391304016, 0.21589618921279907, 0.26951465010643005, 0.20327837765216827, 0.17385198175907135, 0.2015584111213684, 0.21559645235538483, 0.2159324288368225, inf, 0.23625518381595612, 0.2919580638408661, 0.2531993091106415, 0.20137593150138855, 0.25469130277633667, 0.19200864434242249, 0.23981763422489166, 0.2843036651611328, 0.1195162981748581, 0.11953410506248474, 0.2514605224132538, 0.2750541865825653, 0.2580185532569885, 0.20473171770572662, 0.22406511008739471, 0.269441157579422, 0.3426840007305145, 0.26165199279785156, 0.20493555068969727, 0.18093392252922058, 0.17314136028289795, 0.3025035560131073, 0.19469641149044037, 0.2851630747318268, 0.22778765857219696, 0.2365463823080063, 0.2480604350566864, 0.30781033635139465, 0.3192780613899231, 0.24362972378730774, 0.33310404419898987, 0.30389678478240967, 0.23000724613666534, 0.3803051710128784, 0.33251145482063293, 0.3794032633304596, 0.32781195640563965, 0.3271242082118988, 0.41420403122901917, 0.39265725016593933, 0.32511380314826965, 0.3313714265823364, 0.38781771063804626, 0.4390583634376526, 0.36281368136405945, 0.2961282432079315, 0.35538360476493835, 0.4356251358985901, 0.6554577946662903, 0.5741710662841797, 0.6142258048057556, 0.4792741537094116, 0.5918684005737305, 0.510697066783905, 0.5658816695213318, 0.5469260811805725, 0.7224023938179016, 0.5947105884552002, 0.6850616335868835, 0.5386847257614136, 0.4891963303089142, 0.5418421626091003, 0.5460289716720581, 0.42367541790008545, 0.46140149235725403, 0.4188266396522522, 0.49766862392425537, 0.5440701842308044, 0.4196591377258301, 0.489654541015625, 0.4724593460559845, 0.6556914448738098, 0.49883344769477844, 0.512082040309906, 0.45444178581237793, 0.5262277126312256, 0.49501633644104004, 0.5051635503768921, 0.5357401967048645, 0.39169785380363464, 0.41733530163764954, 0.47604215145111084, 0.6164063811302185, 0.5285179018974304, 0.4179104268550873, 0.6063568592071533, 0.5298254489898682, 0.5168486833572388, 0.49945464730262756, 0.5992226600646973, 0.4800785481929779, 0.5385319590568542, 0.592817485332489, 0.2522232234477997, 0.18351902067661285, 0.22462107241153717, 0.1874244660139084, 0.28520888090133667, 0.17797963321208954, 0.21101541817188263, 0.1577177494764328, 0.22699424624443054, 0.18961909413337708, 0.13251399993896484, 0.23423898220062256, 0.15589244663715363, 0.1972072571516037, 0.21768024563789368, 0.18760626018047333, 0.15969528257846832, 0.2100314199924469, 0.1954541951417923, 0.22741180658340454, 0.32197806239128113, 0.23507581651210785, 0.23009908199310303, 0.2720101773738861, 0.279245525598526, 0.18857577443122864, 0.23903542757034302, 0.29033130407333374, 0.31262877583503723, 0.29010817408561707, 0.28770166635513306, 0.28774455189704895, 0.22769273817539215, 0.2340036779642105, 0.22597861289978027, 0.2092037945985794, 0.29487961530685425, 0.26157981157302856, 0.33794981241226196, 0.33055517077445984, 0.5349906086921692, 0.5604430437088013, 0.49731341004371643, 0.6179866194725037, 0.4557833671569824, 0.5561702251434326, 0.5758106708526611, 0.5876657962799072, 0.4739888310432434, 0.532441258430481, 0.6564450263977051, 0.5949026346206665, 0.506523847579956, 0.4962039291858673, 0.5769637227058411, 0.5305290818214417, 0.5243743062019348, 0.5630728602409363, 0.5499339699745178, 0.5426943898200989, 0.5387420058250427, 0.5104788541793823, 0.49283233284950256, 0.5140125155448914, 0.5392475724220276, 0.4911278784275055, 0.4475021958351135, 0.4300402104854584, 0.519229531288147, 0.5853182077407837, 0.5366029739379883, 0.3921224772930145, 0.5604062080383301, 0.480670303106308, 0.33363744616508484, 0.3431345522403717, 0.334419310092926, 0.3427373468875885, 0.3462301790714264, 0.2975237965583801, 0.3570209741592407, 0.3863675892353058, 0.3950467109680176, 0.33298662304878235, 0.1834823042154312, 0.32262203097343445, 0.40367457270622253, 0.24515312910079956, 0.4620848000049591, 0.2201574742794037, 0.27985045313835144, 0.3099226951599121, 0.40429022908210754, 0.33313000202178955, 0.3101198971271515, 0.4300297200679779, 0.2980937361717224, 0.24239006638526917, 0.24915660917758942, 0.23759539425373077, 0.2811908423900604, 0.29158729314804077, 0.23030368983745575, 0.26871180534362793, 0.2554011642932892, 0.196099191904068, 0.29256942868232727, 0.29704535007476807, 0.20476894080638885, 0.24240978062152863, 0.2923740744590759, 0.33973756432533264, 0.2372143715620041, 0.3542623817920685, 0.5632749199867249, 0.7383043169975281, 0.659284770488739, 0.5782424211502075, 0.6068640351295471, 0.4984363317489624, 0.6031405925750732, 0.6880811452865601, 0.5510637760162354, 0.5841869115829468, 0.6659948825836182, 0.6120030283927917, 0.553618848323822, 0.5952198505401611, 0.5126561522483826, 0.5771466493606567, 0.5634941458702087, 0.3984982669353485, 0.5251607894897461, 0.48776742815971375, 0.47470980882644653, 0.5156792998313904, 0.4887528121471405, 0.5855327844619751, 0.5419006943702698, 0.4494587481021881, 0.5726624131202698, 0.5392696261405945, 0.531489908695221, 0.4101172983646393, 0.5423108339309692, 0.5362934470176697, 0.5172243714332581, 0.40829089283943176, 0.49729326367378235, 0.4906744062900543, 0.35267454385757446, 0.5174764394760132, 0.5086653232574463, 0.5791694521903992, 0.3546142578125, 0.3673607409000397, 0.518005907535553, 0.44954994320869446, 0.4691341519355774, 0.44956403970718384, 0.5165873765945435]

x = []
y = []
for i in range(len(points)):
  if points[i] == 0:
    continue
  x.append(points[i] * math.cos(math.radians(i)))
  y.append(points[i] * math.sin(math.radians(i)))

figure(figsize=(8, 6), dpi=80)

plt.scatter(x = x, y = y)
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

"""# **b)**"""

x = []
y = []
for i in range(len(points)):
  if i >= 90:
    break
  if points[i] == 0:
    continue
  x.append(points[i] * math.cos(math.radians(i)))
  y.append(points[i] * math.sin(math.radians(i)))

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

best_line = ransac(x, y, 0.075)

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
for i in range(len(points)):
  if points[i] == 0:
    continue
  x.append(points[i] * math.cos(math.radians(i)))
  y.append(points[i] * math.sin(math.radians(i)))

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
def split_part(X, Y, threshold=0.17, start = 0, end = len(x) - 1):
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
    split_part(X, Y, threshold, start, end= max_arg)
    split_part(X, Y, threshold, start = max_arg + 1, end = end)
  return

split_part(x, y)

breaks = list(breaks)
breaks.sort()

def get_sum_dis(X, Y, a, c):
  dis = distance_line_point(a, -1, c, X, Y)
  return np.sum(dis)

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

#   return  get_sum_dis(x[s:e], y[s:e], a, c) / l

def merge_part(breaks):
  start = 0
  mid, lb = creat_stack(breaks)
  bp = [i for i in breaks]
  i = 0

  # print(len(lb))
  while i<len(lb):
    # print("i is: ",i)
    if MNE(start, lb[i]) < (MNE(start, mid) + MNE(mid, lb[i]) / 2):
      # print('start is: ', start, " mid is: ", mid)
      bp.remove(mid)
      mid = lb[i]
      # print('first')
      
    
    elif (i < len(lb) - 1) and (MNE(mid, lb[i+1]) < (MNE(mid, lb[i]) + MNE(lb[i], lb[i+1])) / 2):
      # print('second')
      # print('start is: ', start, " mid is: ", mid)
      bp.remove(lb[i])
      start = mid
      mid = lb[i+1]

      i += 1

    else:
      # print('third')
      # print('start is: ', start, " mid is: ", mid)
      if (i < len(lb) - 1):
        break
      start = lb[i]
      mid = lb[i+1]
      i+=1
    i += 1
  
  return bp

tmp = merge_part(breaks)

breaks = list(breaks)
breaks.sort()

from matplotlib.pyplot import figure

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

from matplotlib.pyplot import figure

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