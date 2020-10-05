#!/usr/bin/python3

# path fn can be modified by any user to generate CSV and binary path file for any functionally defined path

def path(t):
  """
  for t ranging from 0, to 1
  returns path point
  """
  return [t,0,0]

csvfile = open("path.csv", mode='w', encoding='utf-8')

i = 0
while i <= 1:
  point = path(i)
  
  for elem in point:
    csvfile.write(str(elem) + ", ")
  csvfile.write('\n')
  i += 0.01

csvfile.close()