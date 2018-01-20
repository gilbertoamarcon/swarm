import matplotlib.pyplot as plt
import numpy as np
import sys

# Input Handling 
if len(sys.argv) < 2:
	print '\033[31m[ERROR]\033[0m Missing argument: Please provide the input and output file names.'
	sys.exit(2)
else:
	input_filename	= sys.argv[1]
	output_filename	= sys.argv[2]

x,y,z = np.loadtxt(input_filename, delimiter=',', unpack=True)
n = int(len(x)**0.5)

v = np.linspace(-1.6, 1.6, 9, endpoint=True)
cmap=plt.cm.coolwarm
cp = plt.contourf(x.reshape(n,n), y.reshape(n,n), z.reshape(n,n), v, cmap=cmap)
plt.colorbar(cp)
plt.xlabel('x')
plt.ylabel('y')
plt.grid(which='major', axis='both')
plt.savefig(output_filename)
