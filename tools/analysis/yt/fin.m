% Parameter-variant sweep helper for tracker tuning (see tools/ballstudy, tools/compare3). (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(genpath('/home/seb/soccer-sim'));
R=tune1('FINAL_s1',struct(),[1 1],1,1,400);
R=tune1('FINAL_s3',struct(),[1 1],1,3,800);
