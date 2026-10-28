% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
checkGroundTruthHead(); checkGroundTruthHead('eval/gt/old/real01.csv');
gt=loadGroundTruth('eval/gt/old/real01.csv'); disp(gt.hasHead)
