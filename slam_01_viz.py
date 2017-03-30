from pylab import *
import pandas as pd
import math


# with open('pose.txt') as f:
#     table = pd.read_table(f, sep=' ', lineterminator='\n', header=None)
    
# print(table.head())

# plot(table.values[:,1],table.values[:,2])
# show()

with open('robot4_scan.txt') as f:
    scan_table = pd.read_table(f, sep=' ', lineterminator='\n', header=None)
scans = scan_table.values[235][1:-1]
print(scans)

with open('scan_deriv.txt') as f:
    scan_deriv_table = pd.read_table(f, sep=' ', lineterminator='\n', header=None)
    
# print(scan_table.head())
x1 = range(scan_deriv_table.values.shape[1]);
y1 = scan_deriv_table.values[0]

print(len(x1))
# print(y1)
print(len(scans))

plot(x1,y1,x1,scans)
show()
