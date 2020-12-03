#!/usr/bin/python3
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv
import math
import sys

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

#axs[0].set_xlim(-61, 305)
#axs[0].set_ylim(-61, 305)

path = csvToColumns('paths/path3.4.csv')
ax.plot(path[0], path[1], 'b')


ax.set_aspect(1)
ax.set(xlabel='X (cm)', ylabel='Y (cm)', title='Path 3')
ax.grid()

fig.savefig("path3.png")
