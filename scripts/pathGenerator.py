#!/usr/bin/python3
from array import array

# tunable values
pointSpacingDist = 0.5
tStep = 0.0001


# path fn can be modified by user to generate CSV and binary path file for any functionally defined path

def path(t):
  """
  for t ranging from 0, to 1
  returns path point
  """
  return array('d',[48*t,t,0])

csvfile = open("path.csv", mode='w', encoding='utf-8')
binfile = open("path", mode='wb')

i = 0
distsqr = pointSpacingDist ** 2
lastPoint = array('d',[0,0])

while i <= 1:
  point = path(i)

  tooClose = (point[0] - lastPoint[0]) ** 2 + (point[1] - lastPoint[1]) ** 2 < distsqr

  i += tStep

  if(tooClose):
    continue

  point.tofile(binfile)
  
  for elem in point:
    csvfile.write(str(elem) + ", ")
  csvfile.write('\n')


  lastPoint[0] = point[0]
  lastPoint[1] = point[1]

csvfile.close()
binfile.close()