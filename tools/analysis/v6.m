% Parameter-variant sweep helper for tracker tuning (see tools/ballstudy, tools/compare3). (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(fileparts(mfilename('fullpath')));
A = {'spawnAnalytic',true,'spawnInflate',1,'spawnVelStd',1,'lostStd',3,'minSpawnRange',1.5,'obsCovInflate',16};
V = {mk(A,'lostStd',4), mk(A,'lostStd',6), mk(A,'lostStd',inf), ...
     mk(A,'lostStd',4,'mergeMaxStd',0.6), mk(A,'lostStd',4,'mergeSigma',2), mk(A,'lostStd',4,'minSpawnRange',1.0), mk(A,'lostStd',4,'mergeMinFusions',3)};
R = runTrackerDiag(V, 1:4, 150);
