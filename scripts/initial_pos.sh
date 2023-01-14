#!/bin/bash
jobid=3

filenames=("commands_21Rotations.txt" "commands_21Rotations.txt" "commands_21Rotations_Exchange.txt" "commands_21Rotations_Exchange.txt")
solvers=(0 1 0 1)
filename=${filenames[$jobid]}
solver=${solvers[jobid]}

echo "txt $filename solver $solver"
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
