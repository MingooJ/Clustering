import numpy as np
import matplotlib.pyplot as plt

f1 = open('input1.txt', 'r')
f2 = open('input3_cluster_3.txt', 'r')
i = 0

line_id1 = []
line_x = []
line_y = []
line_id2 = []

while True:
    line1 = f1.readline()
    if line1 == '': break
    
    line1 = line1.split()
    line_id1.append(int(line1[0]))
    line_x.append(float(line1[1]))
    line_y.append(float(line1[2]))


while True:
    line2 = f2.readline()
    if line2 == '': break
    
    line_id2.append(int(line2))
    

for i in range(len(line_id2)):
    index = int(line_id2[i])
    plt.scatter(line_x[index],line_y[index])


f1.close()
f2.close()
plt.axis((0,100,-10,80))
plt.show()
