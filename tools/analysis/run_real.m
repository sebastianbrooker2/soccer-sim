% Run eval/runEvaluation over all 10 real* matches (run_real_cam: camera sensor model variant). (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
files = arrayfun(@(i) sprintf('/home/seb/soccer-sim/eval/gt/real%02d.csv', i), 1:10, 'UniformOutput', false);
r = runEvaluation(struct('gtFiles', {files}, 'runName', 'real_sim', 'params', struct('maxAreaForScore', 3)));
