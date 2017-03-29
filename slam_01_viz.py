from pylab import *
import pandas as pd
import math


with open('pose.txt') as f:
    table = pd.read_table(f, sep=' ', lineterminator='\n', header=None)
    
print(table.head())

plot(table.values[:,1],table.values[:,2])
show()


