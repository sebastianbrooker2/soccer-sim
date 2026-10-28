% Parameter-variant sweep helper for tracker tuning (see tools/ballstudy, tools/compare3). (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
addpath(fileparts(mfilename('fullpath')));
A = {'spawnAnalytic',true,'spawnInflate',1,'spawnVelStd',1,'lostStd',4,'minSpawnRange',1.5,'obsCovInflate',16,'mergeMaxStd',0.6,'mergeSigma',2,'assocLogDet',true};
V = {struct(), mk(A), mk(A,'spawnAnalyticBall',true), mk(A,'obsCovInflateBall',4)};
R = runTrackerDiag(V, 1:4, 150);
