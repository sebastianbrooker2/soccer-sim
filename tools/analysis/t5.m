% Scratch diagnostic/test script from the tracker tuning sessions. (copied from analysis scratchpad; may reference scratch .mat files / absolute paths)
cd('/home/seb/soccer-sim'); addpath(genpath(pwd));
S=load('/tmp/claude-1000/-home-seb-soccer-sim/b40ef6cc-2250-4d87-a6da-409ad2cbd4ac/scratchpad/joint0.mat'); j=S.joint; f=fastCopies(j);
for tt=[100.3 250.7 500 600]
 for c={'ball','robot'}
 a=trackEstimates(j(2),tt,c{1}); b=fastEstimates(f(2),tt,c{1}); fprintf('%d %d\n',numel(a),numel(b));
 end
end
