#!/usr/bin/python3
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv
import math

def csvToColumns(filename):
  points = csv.reader(open(filename))
  columns = []

  for point in points:
    for i in range(len(point)):
        if i >= len(columns):
          columns.append([])

        try:
            columns[i].append(float(point[i]))
        except:
            pass

  return columns

def lineSeg2PointDist(P1x, P1y, P2x, P2y, Rx, Ry):
  projScalar = ((Rx - P1x)*(P2x - P1x) + (Ry - P1y)*(P2y - P1y)) / ((P2x - P1x)*(P2x - P1x) + (P2y - P1y)*(P2y - P1y))

  distsqr = 0

  if projScalar >= 1:
    distsqr = (P2x - Rx) ** 2 + (P2y - Ry) ** 2
  elif projScalar <= 0:
    distsqr = (P1x - Rx) ** 2 + (P1y - Ry) ** 2
  else:
    distsqr = (P1x - Rx) ** 2 + (P1y - Ry) ** 2 - (projScalar ** 2 * ((P2x - P1x)*(P2x - P1x) + (P2y - P1y)*(P2y - P1y)))

  cross = (P2x - P1x) * (Ry - P1y) - (P2y - P1y) * (Rx - P1x)

  return np.sign(cross) * math.sqrt(distsqr)


fig, ax = plt.subplots()

#ax.set_xlim(-61, 305)
#ax.set_ylim(-61, 305)

path = csvToColumns('paths/path1.csv')
ax.plot(path[0], path[1], 'b')

robot = csvToColumns('paths/path2.csv')
ax.plot(robot[0], robot[1], 'r')

ax.set(xlabel='X (cm)', ylabel='Y (cm)', title='Path')
ax.grid()

fig.savefig("analysis/path1Image.png")

lastDistsqr = float('inf')
lastJ = 0
error = []
for i in range(len(robot[0])):
  for j in range(lastJ,len(path[0])):
    pass

print(lineSeg2PointDist(3,-4,20,4,0,0))

plt.show()
