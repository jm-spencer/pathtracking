import csv
import sys
from array import array
import numpy

points = csv.reader(open(str(sys.argv[1])))
binfile = open("binFile", mode='wb')

for point in points:
    p = numpy.array(point)
    p1 = p.astype(numpy.float64)
    p1.tofile(binfile)
