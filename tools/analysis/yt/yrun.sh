#!/bin/bash
D=/tmp/claude-1000/-home-seb-soccer-sim/b40ef6cc-2250-4d87-a6da-409ad2cbd4ac/scratchpad/yt
cd $D
/usr/local/MATLAB/R2026a/bin/matlab -batch "addpath('$D'); R=tune1('$1',$2,$3,$4,${5:-1},${6:-400});" > $D/out_$1.txt 2>&1
