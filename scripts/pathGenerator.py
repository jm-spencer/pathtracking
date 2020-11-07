#!/usr/bin/python3
from array import array
import math

# tunable values
pointSpacingDist = 1
tStep = 0.0001

# i_1 = curve#; i_2 = var#; i_3 = param#
parameters = [
  [[0,10,10,10],[0,0,10,0]],
  [[10,10,20,0],[10,0,0,-10]]
]

# given a hermite curve h defined by params, return h(t)

def hermite(params, t):
  return (2 * (t ** 3) - 3 * (t ** 2) + 1) * params[0] + \
         ((t ** 3) - 2 * (t ** 2) + t) * params[1] + \
         (-2 * (t ** 3) + 3 * (t ** 2)) * params[2] + \
         ((t ** 3) - (t ** 2)) * params[3]

# given a hermite curve h defined by params, return dh/dt (t)

def dHermite(params, t):
  return (6 * (t ** 2) - 6 * t) * params[0] + \
         (3 * (t ** 2) - 4 * t + 1) * params[1] + \
         (-6 * (t ** 2) + 6 * t) * params[2] + \
         (3 * (t ** 2) - 2 * t) * params[3]

# path fn can be modified by user to generate CSV and binary path file for any functionally defined path
# this implimentation uses a parametric cubic hermite spline

def path(t):
  """
  for t ranging from 0, to 1
  returns path point
  """
  if(t != 1):
    t *= len(parameters)
    i = math.floor(t)
    t_i = t % 1
  else:
    i = len(parameters) - 1
    t_i = 1

  return array('d',[hermite(parameters[i][0],t_i),
                    hermite(parameters[i][1],t_i),
                    math.atan2(dHermite(parameters[i][1],t_i),
                               dHermite(parameters[i][0],t_i))])

csvfile = open("path.csv", mode='w', encoding='utf-8')
binfile = open("path", mode='wb')

# record point p to both the binary and the csv file
def record(p):
  p.tofile(binfile)
  for elem in p:
    csvfile.write(str(elem) + ", ")
  csvfile.write('\n')

i = 0
distsqr = pointSpacingDist ** 2
lastPoint = path(0)
record(lastPoint)

while i <= 1:
  point = path(i)

  tooClose = (point[0] - lastPoint[0]) ** 2 + (point[1] - lastPoint[1]) ** 2 < distsqr

  i += tStep

  if(tooClose):
    continue

  record(point)

  lastPoint[0] = point[0]
  lastPoint[1] = point[1]

record(path(1))

csvfile.close()
binfile.close()
