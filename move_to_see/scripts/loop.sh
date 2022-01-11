#!/bin/bash

#echo Input number of runs
#read ulim
#for value in {1..$ulim}
for value in {1..20}
do
    echo Run \# $value
    python3 test_move_to_see_pyrep_singularity.py
done
echo All done
