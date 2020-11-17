#!/usr/bin/python3
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv

parameters = [
  [[0  , 100], [0  ,  0  ]],
  [[200, 200], [0  ,  0  ]],
  [[225, 0  ], [150,  250]],
  [[75 , 0  ], [150, -250]],
  [[125, 0  ], [50 , -150]],
  [[50 , 0  ], [50 ,  150]],
  [[50 , 0  ], [150,  150]],
  [[0  , 0  ], [150, -150]],
  [[0  , 200], [0  ,  0  ]]
]

# given a hermite curve h defined by params, return h(t)

def hermite(params0, params1, t):
  return (2 * (t ** 3) - 3 * (t ** 2) + 1) * params0[0] + \
         ((t ** 3) - 2 * (t ** 2) + t) * params0[1] + \
         (-2 * (t ** 3) + 3 * (t ** 2)) * params1[0] + \
         ((t ** 3) - (t ** 2)) * params1[1]

t = np.arange(0, 1.01, .01)

fig, ax = plt.subplots()

#ax.set_xlim(-61, 305)
#ax.set_ylim(-61, 305)

for i in range(len(parameters) - 1):
  sx = hermite(parameters[i][0], parameters[i+1][0], t)
  sy = hermite(parameters[i][1], parameters[i+1][1], t)

  ax.plot(sx, sy, 'b')

points = csv.reader(open('paths/path2.csv'))

#for point in points:
#    ax.plot(float(point[0]), float(point[1]),'b')

columns = [[],[],[],[]]
for point in points:
    for i in range(4):
        try:
            columns[i].append(float(point[i]))
        except:
            pass

ax.plot(columns[0], columns[1], 'r')

ax.set(xlabel='X (cm)', ylabel='Y (cm)', title='Path')
ax.grid()

fig.savefig("analysis/path1Image.png")
plt.show()
