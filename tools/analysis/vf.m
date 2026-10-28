% Parameter-variant sweep helper for tracker tuning (see tools/ballstudy, tools/compare3). (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(fileparts(mfilename('fullpath')));
O = struct('spawnAnalytic',false,'spawnInflate',4,'spawnVelStd',2.5,'lostStd',inf,'minSpawnRange',0,'obsCovInflate',1,'mergeSigma',3,'mergeMaxStd',1.0,'assocLogDet',false);
V = {O, struct()};
R = runTrackerDiag(V, 1:5, 300);
