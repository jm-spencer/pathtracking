#!/usr/bin/python3
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv
import math
import sys
import json

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
    distsqr = (P1x - Rx) ** 2 + (P1y - Ry) ** 2 - ((projScalar ** 2) * ((P2x - P1x) ** 2 + (P2y - P1y) ** 2))

  # print('<%s,%s>' % (projScalar, distsqr))

  cross = (P2x - P1x) * (Ry - P1y) - (P2y - P1y) * (Rx - P1x)

  # in the event the error is actually 0, rounding errors may cause distsqr to be negative,
  # causing errors when the sqrt is taken
  if abs(distsqr) < 10 ** -12:
    distsqr = 0

  return math.sqrt(distsqr) if cross >= 0 else -1 * math.sqrt(distsqr)

statTable = json.load(open('analysis/stats.json'))

for telemNum in range(1,len(sys.argv)):

    fig, axs = plt.subplots(2, gridspec_kw={'height_ratios': [4, 1]})
    fig.set_size_inches(6.4, 8.2)

    #axs[0].set_xlim(-61, 305)
    #axs[0].set_ylim(-61, 305)

    pathId = str(sys.argv[telemNum])[-5]

    path = csvToColumns('paths/refPath' + str(pathId) + '.csv')
    #path = csvToColumns('paths/ftp' + str(pathId) + '.csv')

    robotFull = csvToColumns(str(sys.argv[telemNum]))

    lastJ = 0
    error = []
    for i in range(len(robotFull[0])):
        minDist = float('inf')
        minimizedIndex = 0
        upperBound = lastJ + 15 if lastJ + 15 < len(path[0]) else len(path[0]) - 1

        for j in range(lastJ, upperBound):
            dist = lineSeg2PointDist(path[0][j], path[1][j], path[0][j+1], path[1][j+1], robotFull[1][i], robotFull[2][i])
            if(abs(dist) < abs(minDist)):
                minimizedIndex = j
                minDist = dist
            #print(str(round(dist, 0)) , end ="," )
        #print()

        if not math.isnan(minDist):
            #print('selecting\t%s (%s, %s)\t%s (%s,%s)-(%s,%s)\t dist = %s' % (i, robotFull[1][i], robotFull[2][i], minimizedIndex, path[0][minimizedIndex], path[1][minimizedIndex], path[0][minimizedIndex+1], path[1][minimizedIndex+1], minDist))
            error.append(minDist)
            lastJ = minimizedIndex

        if lastJ == len(path[0]) - 2:
            break

    robot = [robotRow[:len(error)] for robotRow in robotFull]

    axs[0].plot(path[0], path[1], 'b')
    axs[0].plot(robot[1], robot[2], 'r')
    axs[0].set_aspect(1)
    axs[0].set(xlabel='X (cm)', ylabel='Y (cm)', title=str(sys.argv[telemNum]).split('/')[-1][0:-4])
    axs[0].grid()


    mean = np.mean(error)
    sd   = np.std(error)
    rmse = np.sqrt(np.mean(np.square(error)))
    t    = (robot[0][len(robot[0])-1] - robot[0][0]) / 1000

    statTable[str(sys.argv[telemNum])[5:-4]] = [mean, sd, rmse, t]

    ylim = max(np.abs(error)) * 1.05
    axs[1].plot(range(len(error)), error, 'b')
    axs[1].set_xlim(0, len(error)-1)
    axs[1].set_ylim(-ylim, ylim)
    axs[1].set(xlabel='Point Number\nμ=' + str(mean) + '    σ=' + str(sd) + '\nrmse=' + str(rmse) + '    t=' + str(t), ylabel='Error (cm)')
    axs[1].grid(axis='y')

    fig.savefig('analysis/' + str(sys.argv[telemNum])[5:-4] + '.png')
    #plt.show()

json.dump(statTable, open('analysis/stats.json', 'w'))
