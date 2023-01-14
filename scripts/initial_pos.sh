#!/bin/bash
filename='commands_21Rotations_Exchange.txt'
solver=1
while read line; do
# reading each line and pass arguments to chrono
		echo "Line: $line"
		../build/my_demo ${solver} ${line}
done < $filename

#output_freqsolver=1
#while read line; do
## reading each line and pass arguments to chrono
#		echo "Line: $line"
#		../build/my_demo ${solver} ${line}
#done < $filename
#
#filename='commands_21Rotations_Exchange.txt'
#solver=0
#while read line; do
## reading each line and pass arguments to chrono
#		echo "Line: $line"
#		../build/my_demo ${solver} ${line}
#done < $filename
#
#solver=1
#while read line; do
## reading each line and pass arguments to chrono
#		echo "Line: $line"
#		../build/my_demo ${solver} ${line}
#done < $filename
