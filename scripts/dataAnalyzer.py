#!/usr/bin/python3
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv

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

plt.show()
