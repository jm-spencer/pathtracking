#!/usr/bin/python3
from array import array
import math
import sys

# tunable values
pointSpacingDist = 5
tStep = 0.0001

# i_1 = curve#; i_2 = var#; i_3 = param#
parameters = [
  [[0  , 0  ], [0  , 0  ]],
  [[0  , 0  ], [0  , 0  ]]
]

""" # Path 1 data
parameters = [
  [[0  , 200], [0  ,  0  ]],
  [[200, 200], [0  ,  0  ]],
  [[225, 0  ], [150,  250]],
  [[75 , 0  ], [150, -250]],
  [[125, 0  ], [50 , -150]],
  [[50 , 0  ], [50 ,  150]],
  [[50 , 0  ], [150,  150]],
  [[0  , 0  ], [150, -150]],
  [[0  , 200], [0  ,  0  ]],
  [[200, 200], [0  ,  0  ]]
] # """

""" # Path 2 data
parameters = [
  [[25 , 75  ], [0    , 0  ]],
  [[70 , 37.5], [50   , 65 ]],
  [[130, 37.5], [175  , 65 ]],
  [[175, 75  ], [225  , 0  ]],
  [[225, 0   ], [112.5,-175]],
  [[175,-75  ], [0    , 0  ]],
  [[130,-37.5], [50   , 65 ]],
  [[70 ,-37.5], [175  , 65 ]],
  [[25 ,-75  ], [225  , 0  ]],
  [[-25, 0   ], [112.5,-175]],
  [[25 , 75  ], [0    , 0  ]],
  [[70 , 37.5], [50   , 65 ]]
] # """

""" # Path 3 data
parameters = [
  [[0  , 150], [-50, 0  ]],
  [[200, 0  ], [0  , 150]],
  [[175,-88 ], [200, 88 ]],
  [[125, 0  ], [125,-100]],
  [[100,-150], [50 , 0  ]],
  [[75 , 0  ], [125, 100]],
  [[25 ,-71 ], [200,-71 ]],
  [[0  , 150], [-50, 0  ]],
  [[200, 0  ], [0  , 150]]
] # """

# given a hermite curve h defined by params, return h(t)

def hermite(params0, params1, t):
  return (2 * (t ** 3) - 3 * (t ** 2) + 1) * params0[0] + \
         ((t ** 3) - 2 * (t ** 2) + t) * params0[1] + \
         (-2 * (t ** 3) + 3 * (t ** 2)) * params1[0] + \
         ((t ** 3) - (t ** 2)) * params1[1]

# given a hermite curve h defined by params, return dh/dt (t)

def dHermite(params0, params1, t):
  return (6 * (t ** 2) - 6 * t) * params0[0] + \
         (3 * (t ** 2) - 4 * t + 1) * params0[1] + \
         (-6 * (t ** 2) + 6 * t) * params1[0] + \
         (3 * (t ** 2) - 2 * t) * params1[1]

def d2Hermite(params0, params1, t):
  return (12 * t - 6) * params0[0] + \
         (6  * t - 4) * params0[1] + \
         (-12* t + 6) * params1[0] + \
         (6  * t - 2) * params1[1]

# path fn can be modified by user to generate CSV and binary path file for any functionally defined path
# this implimentation uses a parametric cubic hermite spline

def path(t):
  """
  for t ranging from 0, to 1
  returns path point
  """
  if(t != 1):
    t *= (len(parameters) - 1)
    i = math.floor(t)
    t_i = t % 1
  else:
    i = len(parameters) - 2
    t_i = 1

  x   =   hermite(parameters[i][0], parameters[i+1][0], t_i)
  y   =   hermite(parameters[i][1], parameters[i+1][1], t_i)
  dx  =  dHermite(parameters[i][0], parameters[i+1][0], t_i)
  dy  =  dHermite(parameters[i][1], parameters[i+1][1], t_i)
  d2x = d2Hermite(parameters[i][0], parameters[i+1][0], t_i)
  d2y = d2Hermite(parameters[i][1], parameters[i+1][1], t_i)

  return array('d',[x,
                    y,
                    math.atan2(dy, dx),
                    (dx * d2y - dy * d2x) / (dx ** 2 + dy ** 2) ** 1.5
                    ])

csvfile = open("paths/" + str(sys.argv[1]) + ".csv", mode='w', encoding='utf-8')
binfile = open("paths/" + str(sys.argv[1]), mode='wb')

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
record(array('d', [math.nan, math.nan, math.nan]))

csvfile.close()
binfile.close()
