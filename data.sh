#!/bin/bash
## Generates a plot from the data
cd data
matlab -nodesktop -nosplash -nodisplay -r dataPlot
eog plot.svg
cd ..
