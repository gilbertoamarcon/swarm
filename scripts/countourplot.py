import pandas
import matplotlib.pyplot as plt
import numpy as np
import csv
import sys

# Input Handling 
if len(sys.argv) < 2:
	print '\033[31m[ERROR]\033[0m Missing argument: Please provide the input and output file names.'
	sys.exit(2)
else:
	input_filename	= sys.argv[1]
	output_filename	= sys.argv[2]

data = pandas.read_csv(input_filename, header=None)

# Data extraction
x = list(data[0])
y = list(data[1])
z = data[2]
ncols = int(len(z)**0.5)
z = np.rot90(z.values.reshape([ncols,ncols]))
x = np.linspace(x[-1],x[0],ncols)
y = np.linspace(y[-1],y[0],ncols)


v = np.linspace(-1.6, 1.6, 9, endpoint=True)
cmap=plt.cm.coolwarm
cp = plt.contourf(x, y, z, v, cmap=cmap)
plt.colorbar(cp)
plt.xlabel('x')
plt.ylabel('y')
plt.grid(which='major', axis='both')
plt.savefig(output_filename)
