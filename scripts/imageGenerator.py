#!/usr/bin/python3
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

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

ax.set(xlabel='X (cm)', ylabel='Y (cm)', title='Path')
ax.grid()

fig.savefig("pathImage.png")