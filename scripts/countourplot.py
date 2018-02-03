import matplotlib 
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker
import numpy as np
import sys

axis_name_ag = 'Angle to Goal (rad)'
axis_name_as = 'Angle to Swarm (rad)'
axis_name_dg = 'Distance to Goal (world ratio)'
axis_name_ds = 'Distance to Swarm (world ratio)'

# Input Handling 
if len(sys.argv) < 5:
	print '\033[31m[ERROR]\033[0m Missing argument: Please provide the input and output file names.'
	sys.exit(2)
else:
	input_filename	= sys.argv[1]
	output_filename	= sys.argv[1]
	for i in range(2,6):
		output_filename += '_' + sys.argv[i]
	output_filename += '.svg'

	d_ag = 180*float(sys.argv[2])/np.pi
	d_as = 180*float(sys.argv[3])/np.pi
	d_dg = float(sys.argv[4])
	d_ds = float(sys.argv[5])
	

def sub(input_filename, axarr, px, py, v, cmap, ranges, map, sx, sy):

	x,y,z = np.loadtxt(input_filename, delimiter=',', unpack=True)
	n = int(len(x)**0.5)

	if sx:
		x = np.array([-180+360*(k+np.pi)/(2*np.pi) for k in x])
	if sy:
		y = np.array([-180+360*(k+np.pi)/(2*np.pi) for k in y])
	z = np.array([ranges[0]+(ranges[1]-ranges[0])*(k-map[0])/(map[1]-map[0]) for k in z])

	cp = axarr[px,py].contourf(x.reshape(n,n), y.reshape(n,n), z.reshape(n,n), v, cmap=cmap)

	if sx:
		axarr[px,py].xaxis.set_major_locator(plticker.MultipleLocator(base=60.0))
	if sy:
		axarr[px,py].yaxis.set_major_locator(plticker.MultipleLocator(base=60.0))
	axarr[px,py].grid(which='major', axis='both', color='k', linewidth=0.2, alpha=0.5)
	return cp


def outs(axarr,ranges,map,out,steps,cmap,label):

	v = np.linspace(ranges[0], ranges[1], steps, endpoint=True)

	axarr[2*out+1,0].set(ylabel=axis_name_as)
	axarr[2*out+0,0].set(ylabel=axis_name_dg)

	axarr[2*out+1, 0].axvline(x=d_ag, color='r', linestyle='-', linewidth=0.5)
	axarr[2*out+1, 0].axhline(y=d_as, color='r', linestyle='-', linewidth=0.5)
	cp = sub(input_filename+'_ag_as'+'_%d'%out, axarr, 2*out+1, 0, v, cmap, ranges, map, 1, 1)

	axarr[2*out+0, 0].axvline(x=d_ag, color='r', linestyle='-', linewidth=0.5)
	axarr[2*out+0, 0].axhline(y=d_dg, color='r', linestyle='-', linewidth=0.5)
	cp = sub(input_filename+'_ag_dg'+'_%d'%out, axarr, 2*out+0, 0, v, cmap, ranges, map, 1, 0)

	axarr[2*out+1, 1].axvline(x=d_ds, color='r', linestyle='-', linewidth=0.5)
	axarr[2*out+1, 1].axhline(y=d_as, color='r', linestyle='-', linewidth=0.5)
	cp = sub(input_filename+'_ds_as'+'_%d'%out, axarr, 2*out+1, 1, v, cmap, ranges, map, 0, 1)

	axarr[2*out+0, 1].axvline(x=d_ds, color='r', linestyle='-', linewidth=0.5)
	axarr[2*out+0, 1].axhline(y=d_dg, color='r', linestyle='-', linewidth=0.5)
	cp = sub(input_filename+'_ds_dg'+'_%d'%out, axarr, 2*out+0, 1, v, cmap, ranges, map, 0, 0)

	plt.subplots_adjust(hspace=0.065, wspace=0.065, bottom=0.05, top=0.95, left=0.1, right=0.9)
	cax = plt.axes([0.92, 0.05+(1-out)*0.46, 0.02, 0.1])
	plt.colorbar(cp, cax=cax, label=label)

matplotlib.rcParams.update({'font.size': 6})
matplotlib.rcParams.update({'axes.linewidth': 0.2})
matplotlib.rcParams.update({'xtick.major.width': 0.2})
matplotlib.rcParams.update({'ytick.major.width': 0.2})
plt.text(0.5, 0.5, 'HERE', fontsize=12)
f, axarr = plt.subplots(4,2, sharex='col', sharey='row', squeeze=True)
f.suptitle('%s: %s %s: %s %s: %s %s: %s' % (axis_name_ag, d_ag, axis_name_as, d_as, axis_name_dg, d_dg, axis_name_ds, d_ds))	
outs(axarr=axarr,ranges=[-90.0,90.0],map=[-1.6,1.6],out=0,steps=13,cmap=plt.cm.coolwarm,label='Leader Desired Heading Angle (rad)')
outs(axarr=axarr,ranges=[0.0,1.0],map=[-1.6,1.6],out=1,steps=11,cmap=plt.cm.Greys,label='Leader Desired Velocity (units/sec)')
axarr[3,0].set(xlabel=axis_name_ag)
axarr[3,1].set(xlabel=axis_name_ds)
plt.gcf().set_size_inches(7,12)
plt.savefig(output_filename)
