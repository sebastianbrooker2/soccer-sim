% Parameter-variant sweep helper for tracker tuning (see tools/ballstudy, tools/compare3). (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(fileparts(mfilename('fullpath')));
A = {'spawnAnalytic',true,'spawnInflate',1,'spawnVelStd',1,'lostStd',4,'minSpawnRange',1.5,'obsCovInflate',16,'mergeMaxStd',0.6};
V = {mk(A), mk(A,'mergeSigma',2), mk(A,'mergeSigma',1.5), mk(A,'minSpawnRange',2.0), mk(A,'spawnVelStd',0.5), mk(A,'spawnVelStd',2), mk(A,'robotAccelStd',0.3), mk(A,'assocLogDet',true)};
R = runTrackerDiag(V, 1:4, 150);
