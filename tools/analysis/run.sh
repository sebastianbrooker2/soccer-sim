#!/bin/bash
# usage: run.sh label "struct(...)" "[kb kr]" cScale seed tmax
cd /home/seb/soccer-sim
/usr/local/MATLAB/R2026a/bin/matlab -batch "cd('/tmp/claude-1000/-home-seb-soccer-sim/b40ef6cc-2250-4d87-a6da-409ad2cbd4ac/scratchpad'); R=tune1('$1',$2,$3,$4,${5:-1},${6:-400});" > out_$1.txt 2>&1
