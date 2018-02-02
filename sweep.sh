
# Ranges
steps=+150
a_min=-3.14
a_max=+3.14
d_min=+0.0
d_max=+1.0

d_ag=0.00
d_as=0.00
d_dg=0.00
d_ds=0.00

# Input weights file
w_file=data/Weights/2_WEIGHTS_R100_L8_E100.txt

# Internal Files
viz_dir=data/viz
file_prefix=$viz_dir/sweep

for out in {0..1}; do 

	# Angle-Angle Sweep
	./bin/sweep $steps 0 1 $out $a_min $a_max $a_min $a_max $w_file ${file_prefix}_ag_as_$out $d_ag $d_as $d_dg $d_ds
	# Angle-Distance Sweep
	./bin/sweep $steps 0 2 $out $a_min $a_max $d_min $d_max $w_file ${file_prefix}_ag_ds_$out $d_ag $d_as $d_dg $d_ds
	# Distance-Angle Sweep
	./bin/sweep $steps 3 1 $out $d_min $d_max $a_min $a_max $w_file ${file_prefix}_dg_as_$out $d_ag $d_as $d_dg $d_ds
	# Distance-Distance Sweep
	./bin/sweep $steps 3 2 $out $d_min $d_max $d_min $d_max $w_file ${file_prefix}_dg_ds_$out $d_ag $d_as $d_dg $d_ds

done

# Plotting
python scripts/countourplot.py $file_prefix $d_ag $d_as $d_dg $d_ds
