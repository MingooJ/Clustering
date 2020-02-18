import numpy as np
import matplotlib.pyplot as plt

f = open('input1.txt', 'r')

while True:
	
	line = f.readline()
	if line == '': break

	line = line.split()
	x = float(line[1])
	y = float(line[2])
	plt.scatter(x,y)


f.close()
plt.axis((0,100,-10,80))
plt.show()
